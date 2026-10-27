// Manual driving: browser key state is turned into commands at each control step.
// Up = accelerate, Down = brake, Space = full brake, Left/Right = steer (ramps while held,
// returns toward centre when released). Steering is scaled to the usable angle at the current speed.

import { clamp } from '../sim/geometry.js';

export const MANUAL = Object.freeze({ accel: 2.0, brake: -4.0, steerStep: 0.15, steerReturn: 0.5, steerScale: 0.6 });

export class ManualController {
  constructor() {
    this.name = 'manual (keyboard)';
    this.keys = { up: false, down: false, left: false, right: false, brake: false };
    this.steerFrac = 0;
  }

  setKeys(k) {
    for (const key of Object.keys(this.keys)) if (key in k) this.keys[key] = !!k[key];
  }

  async reset() {
    this.steerFrac = 0;
  }

  async step(obs) {
    const k = this.keys;
    let accel = 0;
    if (k.brake) accel = obs.limits.accelMin;
    else if (k.down) accel = MANUAL.brake;
    else if (k.up) accel = MANUAL.accel;
    const dir = (k.left ? 1 : 0) - (k.right ? 1 : 0);
    if (dir !== 0) this.steerFrac = clamp(this.steerFrac + MANUAL.steerStep * dir, -1, 1);
    else this.steerFrac = Math.abs(this.steerFrac) < 0.02 ? 0 : this.steerFrac * MANUAL.steerReturn;
    return { accel, steer: this.steerFrac * MANUAL.steerScale * obs.steerMaxNow };
  }

  close() {}
}
