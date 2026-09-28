import type { TrackModel } from '../scene/track_model.ts';
import { fixed } from './format.ts';

// The title's second line: what the recorded run covers. Values come from the
// recorded line table and the first frame; loops are used throughout because a
// long record has hundreds of thousands of frames, too many to spread into
// Math.min/Math.max arguments.

export interface Subtitle {
  en: string;
  zh: string;
}

export function scenarioSubtitle(carbodyStation: Float64Array, initialSpeedKmh: number | null, track: TrackModel | null): Subtitle | null {
  let low = Infinity;
  let high = -Infinity;
  for (const station of carbodyStation) {
    if (Number.isFinite(station)) {
      low = Math.min(low, station);
      high = Math.max(high, station);
    }
  }
  const en: string[] = [];
  const zh: string[] = [];
  if (track !== null && low <= high) {
    const sections = track.sections.filter((section) => section.endStation >= low && section.startStation <= high);
    let radius = Infinity;
    let circular = false;
    for (const section of sections) {
      if (section.kind === 'circular') {
        circular = true;
        radius = Math.min(radius, section.radiusMeters ?? Infinity);
      }
    }
    if (circular) {
      let cant = 0;
      track.stations.forEach((station, index) => {
        if (station >= low && station <= high) {
          cant = Math.max(cant, Math.abs(track.cant[index] ?? 0));
        }
      });
      en.push('CURVE NEGOTIATION');
      if (Number.isFinite(radius)) {
        en.push(`R ${fixed(radius, 0)} m`);
      }
      en.push(`CANT ${fixed(1000 * cant, 0)} mm`);
      zh.push('曲线通过');
    } else if (sections.some((section) => section.kind === 'transition')) {
      en.push('TRANSITION CURVE');
      zh.push('缓和曲线');
    } else {
      en.push('TANGENT TRACK');
      zh.push('直线运行');
    }
  }
  if (initialSpeedKmh !== null) {
    en.push(`INITIAL SPEED ${fixed(initialSpeedKmh, 1)} km/h`);
    zh.push(`初速 ${fixed(initialSpeedKmh, 1)} km/h`);
  }
  return en.length === 0 ? null : { en: en.join(' · '), zh: zh.join(' · ') };
}
