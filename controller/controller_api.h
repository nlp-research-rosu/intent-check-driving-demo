/*
 * Controller interface for the driving simulation.
 *
 * A controller is a C translation unit that implements the three functions declared at the bottom
 * of this file. It is linked with controller_host.c into an executable that the simulation backend
 * starts as a child process (see README.md, "Writing your own controller").
 *
 * Units: metres, seconds, radians, m/s, m/s^2.
 * World frame: x points along the highway (direction of travel), y points to the left of travel.
 * heading is measured counter-clockwise from +x. Positive steer turns left.
 * Highway lane k (0 = rightmost) spans y in [k * lane_width, (k + 1) * lane_width].
 * Vehicle positions refer to the centre of the body rectangle.
 *
 * The backend restarts the controller process for every episode (load or reset) and then calls
 * controller_reset() once, so static state does not leak between episodes.
 */
#ifndef CONTROLLER_API_H
#define CONTROLLER_API_H

#ifdef __cplusplus
extern "C" {
#endif

#define CTRL_API_VERSION 1
#define CTRL_MAX_OBJECTS 64
#define CTRL_MAX_REGIONS 16
#define CTRL_MAX_POLY_POINTS 16

/* Kinds of drivable regions (CtrlRegion.kind). */
enum {
    CTRL_REGION_HIGHWAY = 0,
    CTRL_REGION_ENTRANCE_RAMP = 1,
    CTRL_REGION_ACCEL_LANE = 2,
    CTRL_REGION_TAPER = 3,
    CTRL_REGION_DECEL_LANE = 4,
    CTRL_REGION_EXIT_RAMP = 5
};

/* Physical limits of the controlled vehicle. Commands outside
 * [accel_min, accel_max] x [-steer_max, steer_max] are reported as invalid outputs
 * and saturated by the actuator. */
typedef struct {
    double accel_min;      /* m/s^2 (negative: maximum braking) */
    double accel_max;      /* m/s^2 */
    double steer_max;      /* rad, front-wheel angle */
    double steer_rate_max; /* rad/s, actuator slew rate */
    double lat_accel_max;  /* m/s^2, tyre limit: usable steer shrinks with speed */
    double speed_max;      /* m/s */
} CtrlLimits;

/* A convex drivable polygon. The drivable surface is the union of all regions. */
typedef struct {
    int kind;              /* CTRL_REGION_* */
    double speed_limit;    /* m/s */
    int num_points;
    double x[CTRL_MAX_POLY_POINTS];
    double y[CTRL_MAX_POLY_POINTS];
} CtrlRegion;

typedef struct {
    double lane_width;
    int num_lanes;
    double x_min, x_max;   /* extent of the main carriageway */
    int num_regions;
    CtrlRegion regions[CTRL_MAX_REGIONS];
} CtrlRoad;

/* Arrival = ego centre inside this convex polygon. */
typedef struct {
    int num_points;
    double x[CTRL_MAX_POLY_POINTS];
    double y[CTRL_MAX_POLY_POINTS];
    double target_x, target_y; /* representative point inside the polygon */
} CtrlDestination;

typedef struct {
    double x, y, heading, speed;
    double accel;          /* acceleration actually achieved during the last step */
    double steer;          /* current front-wheel angle */
    double length, width, wheelbase;
} CtrlEgo;

/* An observed surrounding vehicle (ground truth within sensor range). */
typedef struct {
    int id;
    double x, y, heading, speed, accel, length, width;
} CtrlObject;

typedef struct {
    double time;           /* simulated time, s */
    long step;             /* physics step index */
    double control_dt;     /* time until the next controller call, s */
    CtrlEgo ego;
    double steer_max_now;  /* usable steering angle at the current speed, rad */
    const CtrlRoad *road;
    const CtrlDestination *destination;
    const CtrlLimits *limits;
    int num_objects;       /* nearest first, at most CTRL_MAX_OBJECTS */
    CtrlObject objects[CTRL_MAX_OBJECTS];
} CtrlInput;

typedef struct {
    double accel;          /* requested longitudinal acceleration, m/s^2 */
    double steer;          /* requested front-wheel angle, rad (positive = left) */
} CtrlOutput;

/* Human-readable controller name (shown in the UI). */
const char *controller_name(void);

/* Called once at the start of every episode. Pointers stay valid for the whole episode. */
void controller_reset(unsigned int seed, const CtrlRoad *road, const CtrlDestination *destination,
                      const CtrlLimits *limits);

/* Called every control step. `out` is pre-filled with NaN; fields left as NaN are invalid outputs. */
void controller_step(const CtrlInput *in, CtrlOutput *out);

#ifdef __cplusplus
}
#endif

#endif /* CONTROLLER_API_H */
