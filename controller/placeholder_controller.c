/*
 * Placeholder controller: requests zero acceleration and zero steering every step, so the vehicle
 * keeps its initial speed and heading. It exists to exercise the interface; autonomous driving
 * behaviour is intentionally left for a later task.
 *
 * To write a real controller, copy this file, fill in controller_step(), and build it with
 *   build_controller.bat path\to\my_controller.c
 */
#include "controller_api.h"

const char *controller_name(void)
{
    return "placeholder (zero command)";
}

void controller_reset(unsigned int seed, const CtrlRoad *road, const CtrlDestination *destination,
                      const CtrlLimits *limits)
{
    (void)seed;
    (void)road;
    (void)destination;
    (void)limits;
}

void controller_step(const CtrlInput *in, CtrlOutput *out)
{
    (void)in;
    out->accel = 0.0;
    out->steer = 0.0;
}
