# Controller interface

Implement three functions from [`controller_api.h`](../controller/controller_api.h):

```c
const char *controller_name(void);
void controller_reset(unsigned int seed, const CtrlRoad *road,
                      const CtrlDestination *destination, const CtrlLimits *limits);
void controller_step(const CtrlInput *in, CtrlOutput *out);
```

`controller_step` runs at 10 Hz. Return acceleration in m/s² and steering angle in radians.
Inputs contain the current time and ego state, road polygons and speed limits, destination,
vehicle limits, and up to 64 observed vehicles within 120 m. Objects are ordered nearest first.
The available steering angle depends on the current speed.

Use the documented observations and controller memory. Although the reset API includes a seed,
the demo challenge forbids using it to predict scripted traffic. The task is to respond to what
the car can observe.

## Build your implementation

Start a new file from `controller/placeholder_controller.c`, then run:

```sh
npm run build:controller -- controller/my_controller.c
```

The default output is `build/controller` on macOS/Linux and `build/controller.exe` on Windows.
You may pass a second argument for another output path:

```sh
npm run build:controller -- controller/my_controller.c build/experiment
```

Set `CONTROLLER_EXE` to use that executable when starting the server. The original
`build_controller.bat` also remains available on Windows.

## Runtime behavior

- The host links with your controller and handles the stdin/stdout protocol.
- Controller output printed to stdout is redirected to stderr, so diagnostic `printf` calls
  cannot corrupt the protocol. Keep logs concise enough to inspect during a run.
- Every load or reset restarts the controller process. Static state starts fresh.
- Output fields are prefilled with NaN. Unset or invalid commands are recorded by the checker.
- A response taking longer than two seconds ends the episode with `invalid_output`.
- The visualizer already shows raw and applied acceleration/steering and checker events.

Keep the simulation, protocol, traffic, observations, and checker intact while solving the
challenge. Add your controller and tests separately so failures remain visible.

## Interpreting results

The supplied suite checks the simulation and protocol, using independent placeholder and
invalid-output executables. Add tests specific to your controller, including boundary cases and
failures found during Live review. Run multiple scenarios and seeds; a successful single run is
limited evidence. Report each scenario's outcome and safety events alongside any assumptions
your controller requires.
