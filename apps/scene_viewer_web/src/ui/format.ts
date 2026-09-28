// Display formatting only: unit changes and rounding, never new quantities.

const minus = '−';

/** Rounds, removes negative zero and uses a true minus sign. */
export function fixed(value: number, digits: number): string {
  const rounded = Number(value.toFixed(digits));
  const clean = rounded === 0 ? 0 : rounded;
  return clean.toFixed(digits).replace('-', minus);
}

/** A signed value with an explicit plus sign. */
export function signed(value: number, digits: number): string {
  const text = fixed(value, digits);
  return text.startsWith(minus) || Number(value.toFixed(digits)) === 0 ? text : `+${text}`;
}

/** Chainage in the K<km>+<metres> form used on Chinese railways. */
export function chainage(stationMeters: number, digits = 1): string {
  const negative = stationMeters < 0;
  const magnitude = Math.abs(stationMeters);
  const kilometres = Math.floor(magnitude / 1000);
  const metres = magnitude - 1000 * kilometres;
  const width = digits > 0 ? 4 + digits : 3;
  return `${negative ? minus : ''}K${kilometres}+${metres.toFixed(digits).padStart(width, '0')}`;
}

/** Minutes, seconds and hundredths. */
export function clock(seconds: number): string {
  const whole = Math.max(0, seconds);
  const minutes = Math.floor(whole / 60);
  const rest = whole - 60 * minutes;
  return `${String(minutes).padStart(2, '0')}:${rest.toFixed(2).padStart(5, '0')}`;
}

/** The smallest of 1, 1.2, 1.5, 2, 2.5, 3, 4, 5, 6, 8 times a power of ten not below x. */
export function niceCeiling(x: number): number {
  if (!(x > 0)) {
    return 1;
  }
  const steps = [1, 1.2, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10];
  const power = 10 ** Math.floor(Math.log10(x));
  for (const step of steps) {
    const candidate = step * power;
    if (candidate >= x * (1 - 1e-12)) {
      return candidate;
    }
  }
  return 10 * power;
}
