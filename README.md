# Intent Check · Highway controller

Build a small autonomous driving controller with your coding agent and Intent Check Live.
Watch it handle merging, sudden braking, lane changes, and a busy exit in a local browser.

The simulator is ready; the controller is deliberately unfinished. Its starting implementation
requests zero acceleration and zero steering. Your agent's task is to replace it with a C
controller while Live reviews the work for subtle logic errors and edge cases.

## Try it with your agent

Open the [Live demo prompt](docs/LIVE_PROMPT.md) and paste it into Claude Code or Codex.
The prompt covers Intent Check setup, the controller task, tests, and opening the simulator.

You can also get the simulator running yourself:

```sh
git clone https://github.com/nlp-research-rosu/intent-check-driving-demo.git
cd intent-check-driving-demo
npm run build:controller
npm test
npm start
```

Open <http://localhost:8080>. Choose a scenario, press **Load scenario**, then **Start**.
With the starting controller, the car keeps its initial speed and heading. Use **Manual
(keyboard)** to explore the road before writing code.

## Requirements

- Node.js 20 or newer. The project has no npm dependencies.
- A C compiler: GCC or Clang on Linux, Xcode Command Line Tools on macOS, or Visual Studio
  Build Tools with the C++ workload on Windows. MinGW-w64 GCC and LLVM Clang are also supported
  on Windows.
- Claude Code or Codex with Intent Check for the Live workflow.

The same npm commands work on Windows, macOS, and Linux. On Windows, the build locates Visual
Studio automatically. If PowerShell blocks `npm.ps1`, use `npm.cmd` in place of `npm`.

## The controller challenge

Implement `controller/my_controller.c` using [the existing C interface](controller/controller_api.h).
The car should merge from the entrance ramp, drive among traffic, and reach its designated exit.
Avoid collisions, stay inside the road, respect speed limits, and make steady progress. When
progress would require an unsafe maneuver, prioritize safety.

Keep the simulation, traffic model, observations, and safety checker unchanged. Decisions must
use current observations and controller memory, without reading scenario seeds or future events.

```sh
# After creating controller/my_controller.c:
npm run build:controller -- controller/my_controller.c
npm test
npm start
```

Add regression tests for the controller and report results across the supplied scenarios and
several seeds. Explain assumptions and remaining failures. See [the controller guide](docs/CONTROLLER.md)
and [the simulator assumptions](docs/ASSUMPTIONS.md).

## Five driving situations

| Scenario | Challenge |
| --- | --- |
| `merge_busy` | Join dense highway traffic from an entrance ramp. |
| `slow_vehicle` | Handle a slow truck ahead. |
| `cut_in` | React to a neighboring car moving into a short gap. |
| `sudden_brake` | Respond when the lead car brakes hard. |
| `exit_traffic` | Reach the exit from the left lane with traffic alongside. |

Each scenario has a seed and adjustable parameters. **Reset** reproduces the same starting
conditions. The independent checker records collisions, road departures, speed violations,
invalid outputs, arrival, and timeouts.

## Controls and feedback

- The ego car is yellow, other cars are blue-gray, and trucks are brown. Its trajectory is cyan
  and the destination is green.
- **Start**, **Pause**, **Step**, and **Reset** control playback. Speed ranges from ×0.25 to ×16.
- The side panel shows speed, steering, acceleration, safety events, and the final outcome.
- For manual driving, focus the road view and use the arrow keys. Space applies full braking.
- Scroll over the road to zoom. The lower strip shows the entire road.

## Tests and configuration

`npm test` builds separate native test fixtures and runs the Node test suite. It covers vehicle
dynamics, deterministic scenarios, traffic behavior, safety events, and the native controller
protocol. These fixtures do not overwrite your controller executable.

| Environment variable | Default | Purpose |
| --- | --- | --- |
| `PORT` | `8080` | Local HTTP port. |
| `HOST` | `127.0.0.1` | Server bind address. |
| `CONTROLLER_EXE` | `build/controller` (`.exe` on Windows) | Controller to run. |
| `CC` | First available `cc`, `gcc`, or `clang` | Compiler executable on macOS/Linux. |

## Project map

```text
controller/       C API, protocol host, and starting controller
server/sim/       Simulation, road, traffic, scenarios, and safety checker
server/           Local HTTP server and controller adapters
public/           Canvas visualization and controls
test/             Simulator and controller protocol tests
scripts/          Portable build entry point
docs/             Controller task, API guide, and model assumptions
```

This is a toy simulation for exploring code and intent. Passing its scenarios describes behavior
within the documented model; it does not establish real-world driving safety.
