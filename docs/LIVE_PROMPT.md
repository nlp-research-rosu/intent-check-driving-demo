# Try Intent Check Live

Copy the prompt below into Claude Code or Codex. First-time setup may require a browser sign-in
and an agent restart. After a restart, ask your agent to resume the saved task.

```text
Help me try Intent Check Live by building a controller for a toy highway simulator.

1. Set up Intent Check first. Read the current installation instructions at
   https://github.com/nlp-research-rosu/intent-check and follow the instructions for
   this agent and operating system. Install the intent plugin if it is missing,
   follow its onboarding instructions to install the CLI and sign in, and run its
   doctor/health checks. Let me complete browser authentication when needed.
   If loading the plugin or trusting its hooks requires reloading or restarting
   this agent, save this task in a local file, tell me the exact next step, and
   resume only after the plugin is loaded and authentication is healthy.

2. Clone https://github.com/nlp-research-rosu/intent-check-driving-demo into a new
   working directory, or use this repository if it is already open. Read README.md,
   docs/ASSUMPTIONS.md, and docs/CONTROLLER.md. Check for Node.js and a C compiler,
   build the starting controller, and run the existing tests.

3. Turn on Intent Check Live for this repository before changing the controller.
   In Claude Code use /intent:live. In Codex follow the installed Intent Check
   skill's Live instructions and confirm that the review hooks are active.

4. Implement controller/my_controller.c using the existing C interface. The car
   should merge from the entrance ramp, drive among traffic, and reach its exit.
   Avoid collisions, stay within the road, obey speed limits, and make steady
   progress. Prioritize safety when immediate progress needs an unsafe maneuver.
   Handle busy merges, slow vehicles, cut-ins, sudden braking, and exits with
   traffic alongside. Base decisions on current observations and controller
   memory, without using scenario seeds or future traffic events. Keep the
   simulation, traffic model, observations, protocol, and safety checker unchanged.

5. Work with Live's review feedback as you implement the controller. Investigate
   reported edge cases, fix confirmed issues, and add regression tests. Build with
   npm run build:controller -- controller/my_controller.c. Run npm test and
   evaluate the controller across all five scenarios and several seeds, reporting
   arrival, collisions, road departures, speed violations, and timeouts honestly.

6. Start the local simulator with npm start and give me its browser URL so I can
   watch the car. Summarize the controller's decisions, the issues Live found,
   assumptions, remaining failures, and the limits of the tested conditions.
```
