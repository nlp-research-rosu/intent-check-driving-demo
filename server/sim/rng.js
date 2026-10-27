// Seeded deterministic PRNG (mulberry32). Never use Math.random in the simulation.

export const SEED_MAX = 0xffffffff;

export function isValidSeed(seed) {
  return Number.isInteger(seed) && seed >= 0 && seed <= SEED_MAX;
}

export function makeRng(seed) {
  let a = seed >>> 0;
  const next = () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  return {
    next,
    uniform: (lo, hi) => lo + (hi - lo) * next(),
    int: (lo, hi) => lo + Math.floor(next() * (hi - lo + 1)),
    chance: (p) => next() < p,
    /** Exponentially distributed sample with the given mean. */
    exp: (mean) => -mean * Math.log(1 - next()),
  };
}
