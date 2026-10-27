// Global simulation constants. All units SI: metres, seconds, radians, m/s, m/s^2.

/** Fixed physics timestep. Simulation results depend only on this, never on rendering. */
export const PHYS_DT = 0.02;
/** The controller is queried every CONTROL_PERIOD_STEPS physics steps (zero-order hold between). */
export const CONTROL_PERIOD_STEPS = 5;
export const CONTROL_DT = PHYS_DT * CONTROL_PERIOD_STEPS;

/**
 * Actuator/plant limits of the controlled (ego) vehicle.
 * accelMin/accelMax and steerMax are also the valid command range checked by the safety checker.
 * latAccelMax models tyre friction: the achievable steering angle shrinks with speed so that
 * v^2 / wheelbase * tan(steer) <= latAccelMax.
 */
export const EGO_LIMITS = Object.freeze({
  accelMin: -8.0,
  accelMax: 3.0,
  steerMax: 0.5,
  steerRateMax: 0.7,
  latAccelMax: 8.0,
  speedMax: 45.0,
});

export const CAR_LIMITS = Object.freeze({
  accelMin: -9.0,
  accelMax: 2.5,
  steerMax: 0.5,
  steerRateMax: 0.5,
  latAccelMax: 4.0,
  speedMax: 45.0,
});

export const TRUCK_LIMITS = Object.freeze({
  accelMin: -7.0,
  accelMax: 1.2,
  steerMax: 0.5,
  steerRateMax: 0.3,
  latAccelMax: 3.0,
  speedMax: 30.0,
});

export const DIMS = Object.freeze({
  ego: Object.freeze({ length: 4.7, width: 1.9, wheelbase: 2.85 }),
  car: Object.freeze({ length: 4.6, width: 1.9, wheelbase: 2.8 }),
  truck: Object.freeze({ length: 12.0, width: 2.55, wheelbase: 7.5 }),
});

/** Surrounding traffic is observable within this radius (centre-to-centre) of the ego. */
export const SENSOR_RANGE = 120;
/** At most this many observed objects are passed to the controller (nearest first). */
export const MAX_OBJECTS = 64;

/** Speed above limit + tolerance counts as a speed violation. */
export const SPEED_TOLERANCE = 0.1;
/** Maximum relative displacement between collision/departure samples within one physics step. */
export const SWEEP_SPACING = 0.1;
/** Footprint boundary sampling spacing for road-departure checks. */
export const FOOTPRINT_EDGE_SPACING = 1.0;

/** Rules used to reject invalid starting configurations. */
export const VALIDATION = Object.freeze({
  /** Minimum bumper-to-bumper gap at standstill between same-lane vehicles. */
  minGap: 2.0,
  /** Reaction time used for the initial safe-gap requirement. */
  reactionTime: 0.3,
  /** Deceleration assumed available to a follower when checking the initial gap. */
  brakeDecel: 6.0,
  /** Footprints are inflated by this margin when checking for initial overlap. */
  overlapMargin: 0.5,
  maxVehicles: 400,
  timeLimitMin: 5,
  timeLimitMax: 600,
});
