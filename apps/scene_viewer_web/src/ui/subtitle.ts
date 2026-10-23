import { chainage } from './format.ts';

// Record coverage belongs in the title; current radius, cant and speed belong
// in the track card. A long route is not represented by a single curve radius.

export interface Subtitle {
  en: string;
  zh: string;
}

export function recordRangeSubtitle(carbodyStation: Float64Array): Subtitle | null {
  let low = Infinity;
  let high = -Infinity;
  for (const stationMeters of carbodyStation) {
    if (Number.isFinite(stationMeters)) {
      low = Math.min(low, stationMeters);
      high = Math.max(high, stationMeters);
    }
  }
  return low <= high ? { en: `RECORDED RANGE · ${chainage(low)} – ${chainage(high)}`, zh: '回放里程范围' } : null;
}
