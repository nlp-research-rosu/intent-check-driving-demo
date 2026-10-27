/*
 * Controller host: links against a controller implementing controller_api.h and talks to the
 * simulation backend over stdin/stdout with a line-based text protocol. You normally do not need
 * to modify this file.
 *
 * Protocol (one request line in, one response line out; numbers use round-trip precision):
 *   on start         -> "READY <api_version> <controller name>"
 *   R <seed> <control_dt> <limits:6> <lane_width> <num_lanes> <x_min> <x_max>
 *     <num_regions> { <kind> <speed_limit> <n> { <x> <y> }*n }*
 *     <dest_n> { <x> <y> }*dest_n <target_x> <target_y>
 *                    -> "OK"            (controller_reset called)
 *   S <time> <step> <ego: x y heading speed accel steer length width wheelbase> <steer_max_now>
 *     <num_objects> { <id> <x> <y> <heading> <speed> <accel> <length> <width> }*
 *                    -> "C <accel> <steer>"   (controller_step called)
 *   Q                -> process exits
 *   malformed input  -> "E <message>"
 *
 * Anything the controller prints to stdout is redirected to stderr so it cannot corrupt the protocol.
 */
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "controller_api.h"

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#define DUP _dup
#define DUP2 _dup2
#define FDOPEN _fdopen
#define FILENO _fileno
#else
#include <unistd.h>
#define DUP dup
#define DUP2 dup2
#define FDOPEN fdopen
#define FILENO fileno
#endif

#define LINE_MAX_LEN (1 << 17)

static char g_line[LINE_MAX_LEN];
static CtrlRoad g_road;
static CtrlDestination g_dest;
static CtrlLimits g_limits;
static double g_control_dt;
static CtrlInput g_input;
static int g_have_episode = 0;

typedef struct {
    const char *p;
    int ok;
} Parser;

static double next_num(Parser *ps)
{
    char *end;
    double v;
    while (isspace((unsigned char)*ps->p)) ps->p++;
    if (*ps->p == '\0') {
        ps->ok = 0;
        return 0.0;
    }
    v = strtod(ps->p, &end);
    if (end == ps->p) {
        ps->ok = 0;
        return 0.0;
    }
    ps->p = end;
    return v;
}

static int next_int(Parser *ps, long lo, long hi)
{
    double v = next_num(ps);
    if (!ps->ok || v != floor(v) || v < (double)lo || v > (double)hi) {
        ps->ok = 0;
        return 0;
    }
    return (int)v;
}

static int at_end(Parser *ps)
{
    while (isspace((unsigned char)*ps->p)) ps->p++;
    return *ps->p == '\0';
}

static int parse_reset(Parser *ps, unsigned int *seed)
{
    int i, k;
    double s = next_num(ps);
    if (!ps->ok || s < 0 || s > 4294967295.0 || s != floor(s)) return 0;
    *seed = (unsigned int)s;
    g_control_dt = next_num(ps);
    g_limits.accel_min = next_num(ps);
    g_limits.accel_max = next_num(ps);
    g_limits.steer_max = next_num(ps);
    g_limits.steer_rate_max = next_num(ps);
    g_limits.lat_accel_max = next_num(ps);
    g_limits.speed_max = next_num(ps);
    g_road.lane_width = next_num(ps);
    g_road.num_lanes = next_int(ps, 1, 16);
    g_road.x_min = next_num(ps);
    g_road.x_max = next_num(ps);
    g_road.num_regions = next_int(ps, 0, CTRL_MAX_REGIONS);
    for (i = 0; ps->ok && i < g_road.num_regions; i++) {
        CtrlRegion *r = &g_road.regions[i];
        r->kind = next_int(ps, 0, 100);
        r->speed_limit = next_num(ps);
        r->num_points = next_int(ps, 3, CTRL_MAX_POLY_POINTS);
        for (k = 0; ps->ok && k < r->num_points; k++) {
            r->x[k] = next_num(ps);
            r->y[k] = next_num(ps);
        }
    }
    g_dest.num_points = next_int(ps, 3, CTRL_MAX_POLY_POINTS);
    for (k = 0; ps->ok && k < g_dest.num_points; k++) {
        g_dest.x[k] = next_num(ps);
        g_dest.y[k] = next_num(ps);
    }
    g_dest.target_x = next_num(ps);
    g_dest.target_y = next_num(ps);
    return ps->ok && at_end(ps);
}

static int parse_step(Parser *ps, CtrlInput *in)
{
    int i;
    in->time = next_num(ps);
    in->step = (long)next_int(ps, 0, 2000000000L);
    in->control_dt = g_control_dt;
    in->ego.x = next_num(ps);
    in->ego.y = next_num(ps);
    in->ego.heading = next_num(ps);
    in->ego.speed = next_num(ps);
    in->ego.accel = next_num(ps);
    in->ego.steer = next_num(ps);
    in->ego.length = next_num(ps);
    in->ego.width = next_num(ps);
    in->ego.wheelbase = next_num(ps);
    in->steer_max_now = next_num(ps);
    in->road = &g_road;
    in->destination = &g_dest;
    in->limits = &g_limits;
    in->num_objects = next_int(ps, 0, CTRL_MAX_OBJECTS);
    for (i = 0; ps->ok && i < in->num_objects; i++) {
        CtrlObject *o = &in->objects[i];
        o->id = next_int(ps, 0, 2000000000L);
        o->x = next_num(ps);
        o->y = next_num(ps);
        o->heading = next_num(ps);
        o->speed = next_num(ps);
        o->accel = next_num(ps);
        o->length = next_num(ps);
        o->width = next_num(ps);
    }
    return ps->ok && at_end(ps);
}

int main(void)
{
    FILE *proto;
    int proto_fd;

#ifdef _WIN32
    _setmode(FILENO(stdin), _O_BINARY);
#endif
    /* Keep the real stdout for the protocol; send everything else printed to stdout to stderr. */
    fflush(stdout);
    proto_fd = DUP(FILENO(stdout));
    if (proto_fd < 0) return 2;
    proto = FDOPEN(proto_fd, "wb");
    if (!proto) return 2;
    DUP2(FILENO(stderr), FILENO(stdout));

    fprintf(proto, "READY %d %s\n", CTRL_API_VERSION, controller_name());
    fflush(proto);

    while (fgets(g_line, sizeof g_line, stdin)) {
        Parser ps;
        size_t len = strlen(g_line);
        if (len == sizeof g_line - 1 && g_line[len - 1] != '\n') {
            int c;
            while ((c = fgetc(stdin)) != EOF && c != '\n') {}
            fprintf(proto, "E line too long\n");
            fflush(proto);
            continue;
        }
        ps.p = g_line + 1;
        ps.ok = 1;
        switch (g_line[0]) {
        case 'R': {
            unsigned int seed = 0;
            if (!parse_reset(&ps, &seed)) {
                fprintf(proto, "E malformed reset request\n");
            } else {
                controller_reset(seed, &g_road, &g_dest, &g_limits);
                g_have_episode = 1;
                fprintf(proto, "OK\n");
            }
            break;
        }
        case 'S': {
            CtrlOutput out;
            if (!g_have_episode) {
                fprintf(proto, "E step before reset\n");
            } else if (!parse_step(&ps, &g_input)) {
                fprintf(proto, "E malformed step request\n");
            } else {
                out.accel = NAN;
                out.steer = NAN;
                controller_step(&g_input, &out);
                fprintf(proto, "C %.17g %.17g\n", out.accel, out.steer);
            }
            break;
        }
        case 'Q':
            fflush(proto);
            return 0;
        default:
            fprintf(proto, "E unknown request\n");
            break;
        }
        fflush(proto);
        fflush(stdout);
    }
    return 0;
}
