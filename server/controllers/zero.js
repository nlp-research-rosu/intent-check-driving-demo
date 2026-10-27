// In-process equivalent of the placeholder C controller (zero command). Used by tests.

export class ZeroController {
  constructor() {
    this.name = 'zero (JS)';
  }
  async reset() {}
  async step() {
    return { accel: 0, steer: 0 };
  }
  close() {}
}
