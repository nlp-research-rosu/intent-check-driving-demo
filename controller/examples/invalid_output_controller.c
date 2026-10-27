/*
 * Example controller that deliberately misbehaves, to demonstrate the safety checker's
 * invalid-output reporting:
 *   t <  2 s : valid commands (zero)
 *   t <  4 s : acceleration above accel_max (reported, actuator saturates)
 *   t >= 4 s : NaN steering (reported, episode ends)
 *
 * Build with:  build_controller.bat controller\examples\invalid_output_controller.c build\invalid_controller.exe
 */
#include <math.h>
#include "controller_api.h"

static const CtrlLimits *g_limits;

const char *controller_name(void)
{
    return "invalid-output demo";
}

void controller_reset(unsigned int seed, const CtrlRoad *road, const CtrlDestination *destination,
                      const CtrlLimits *limits)
{
    (void)seed;
    (void)road;
    (void)destination;
    g_limits = limits;
}

void controller_step(const CtrlInput *in, CtrlOutput *out)
{
    out->accel = 0.0;
    out->steer = 0.0;
    if (in->time >= 4.0) {
        out->steer = NAN;
    } else if (in->time >= 2.0) {
        out->accel = g_limits->accel_max + 5.0;
    }
}
