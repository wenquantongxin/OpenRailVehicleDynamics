import * as THREE from 'three';

import type { TrackTable } from '../record/scene_record.ts';

// Reads the line table that ORVD sampled into the record. Everything here is in
// the track inertial frame I (x along increasing station at the line origin,
// y right, z down). The recorded Track-T rotation already contains the cant
// roll, so a rail or sleeper section written in Track-T coordinates only needs
// the station's origin and rotation.
//
// Sections are recovered from the sampled curvature for display: tangent where
// |k| is below 1e-4 of the largest curvature, circular where k stays on a
// plateau (each plateau keeps its own radius), transition elsewhere. A run
// shorter than 4 m between two runs of the same kind and radius is absorbed.
// On the bundled R300 line this places the element points within 0.3 m of the
// line definition; the record carries no segment table.
//
// Rail positions come from the recorded rail datums, expressed per station in
// Track-T about the centreline, so a different gauge or datum is drawn as
// recorded.

export type SectionKind = 'tangent' | 'transition' | 'circular';

export interface TrackSection {
  kind: SectionKind;
  startStation: number;
  endStation: number;
  /** Plateau radius of a circular section; null otherwise. */
  radiusMeters: number | null;
  /** +1 turns right (towards +y), -1 turns left, 0 straight. */
  direction: number;
}

export interface ElementPoint {
  code: 'TS' | 'SC' | 'CS' | 'ST' | 'TC' | 'CT' | 'CC';
  zh: string;
  station: number;
}

const elementNames: Record<ElementPoint['code'], string> = {
  TS: '直缓点',
  SC: '缓圆点',
  CS: '圆缓点',
  ST: '缓直点',
  TC: '直圆点',
  CT: '圆直点',
  CC: '复曲线点',
};

export interface TrackFrame {
  position: THREE.Vector3;
  /** Columns: along track, right, down (Track-T axes expressed in I). */
  tangent: THREE.Vector3;
  right: THREE.Vector3;
  down: THREE.Vector3;
}

export class TrackModel {
  readonly stations: Float64Array;
  readonly curvature: Float64Array;
  readonly cant: Float64Array;
  readonly sections: TrackSection[];
  readonly elementPoints: ElementPoint[];
  /** Recorded rail datums in Track-T about the centreline: lateral v (right) and depth w (down). */
  readonly leftRailV: Float64Array;
  readonly leftRailW: Float64Array;
  readonly rightRailV: Float64Array;
  readonly rightRailW: Float64Array;
  /** The lowest centreline point of the table, as inertial z (z points down). */
  readonly lowestCentreZ: number;
  /** Median distance between the two rail datums. */
  readonly gauge: number;
  private readonly centers: THREE.Vector3[];
  private readonly rotations: THREE.Quaternion[];
  private hint = 0;

  constructor(table: TrackTable) {
    const count = table.stationsMeters.length;
    this.stations = Float64Array.from(table.stationsMeters);
    this.curvature = Float64Array.from(table.curvatureRadiansPerMeter);
    this.cant = Float64Array.from(table.superelevationMeters);
    this.centers = table.centerlineInInertialMeters.map((row) => new THREE.Vector3(row[0] ?? 0, row[1] ?? 0, row[2] ?? 0));
    this.rotations = table.rotationInertialFromTrackWxyz.map((row) =>
      new THREE.Quaternion(row[1] ?? 0, row[2] ?? 0, row[3] ?? 0, row[0] ?? 1).normalize(),
    );
    if (
      this.curvature.length !== count ||
      this.rotations.length !== count ||
      this.cant.length !== count ||
      table.leftRailDatumInInertialMeters.length !== count ||
      table.rightRailDatumInInertialMeters.length !== count
    ) {
      throw new Error('track table: curvature, cant, rotation and rail columns disagree with the stations');
    }
    this.leftRailV = new Float64Array(count);
    this.leftRailW = new Float64Array(count);
    this.rightRailV = new Float64Array(count);
    this.rightRailW = new Float64Array(count);
    const offset = new THREE.Vector3();
    const right = new THREE.Vector3();
    const down = new THREE.Vector3();
    let lowest = -Infinity;
    const gauges = new Float64Array(count);
    for (let index = 0; index < count; ++index) {
      const centre = this.centers[index] as THREE.Vector3;
      const rotation = this.rotations[index] as THREE.Quaternion;
      right.set(0, 1, 0).applyQuaternion(rotation);
      down.set(0, 0, 1).applyQuaternion(rotation);
      const left = table.leftRailDatumInInertialMeters[index] ?? [];
      offset.set(left[0] ?? 0, left[1] ?? 0, left[2] ?? 0).sub(centre);
      this.leftRailV[index] = offset.dot(right);
      this.leftRailW[index] = offset.dot(down);
      const rightRail = table.rightRailDatumInInertialMeters[index] ?? [];
      offset.set(rightRail[0] ?? 0, rightRail[1] ?? 0, rightRail[2] ?? 0).sub(centre);
      this.rightRailV[index] = offset.dot(right);
      this.rightRailW[index] = offset.dot(down);
      gauges[index] = (this.rightRailV[index] as number) - (this.leftRailV[index] as number);
      lowest = Math.max(lowest, centre.z);
    }
    this.lowestCentreZ = count > 0 ? lowest : 0;
    gauges.sort();
    this.gauge = count > 0 ? (gauges[count >> 1] as number) : 1.435;
    this.sections = classifySections(this.stations, this.curvature);
    this.elementPoints = elementPointsOf(this.sections);
  }

  get count(): number {
    return this.stations.length;
  }

  get firstStation(): number {
    return this.stations[0] as number;
  }

  get lastStation(): number {
    return this.stations[this.stations.length - 1] as number;
  }

  /** Fractional table index of a station, clamped to the table. */
  indexOfStation(station: number): number {
    const s = this.stations;
    const last = s.length - 1;
    if (station <= (s[0] as number)) {
      return 0;
    }
    if (station >= (s[last] as number)) {
      return last;
    }
    let low = 0;
    let high = last;
    while (high - low > 1) {
      const middle = (low + high) >> 1;
      if ((s[middle] as number) <= station) {
        low = middle;
      } else {
        high = middle;
      }
    }
    const s0 = s[low] as number;
    const s1 = s[high] as number;
    return low + (station - s0) / (s1 - s0);
  }

  private interpolate(values: Float64Array, station: number): number {
    const index = this.indexOfStation(station);
    const low = Math.floor(index);
    const high = Math.min(values.length - 1, low + 1);
    const fraction = index - low;
    return (values[low] as number) * (1 - fraction) + (values[high] as number) * fraction;
  }

  /** Recorded rail datum of one side at a station, in Track-T about the centreline: [v, w]. */
  railOffsetAt(station: number, side: 'left' | 'right'): [number, number] {
    const v = side === 'left' ? this.leftRailV : this.rightRailV;
    const w = side === 'left' ? this.leftRailW : this.rightRailW;
    return [this.interpolate(v, station), this.interpolate(w, station)];
  }

  curvatureAt(station: number): number {
    return this.interpolate(this.curvature, station);
  }

  cantAt(station: number): number {
    return this.interpolate(this.cant, station);
  }

  sectionAt(station: number): TrackSection | null {
    for (const section of this.sections) {
      if (station >= section.startStation && station <= section.endStation) {
        return section;
      }
    }
    return null;
  }

  /** Origin and Track-T axes at a station, interpolated between samples. */
  frameAt(station: number, out?: TrackFrame): TrackFrame {
    const frame = out ?? {
      position: new THREE.Vector3(),
      tangent: new THREE.Vector3(),
      right: new THREE.Vector3(),
      down: new THREE.Vector3(),
    };
    const index = this.indexOfStation(station);
    const low = Math.floor(index);
    const high = Math.min(this.count - 1, low + 1);
    const fraction = index - low;
    const c0 = this.centers[low] as THREE.Vector3;
    const c1 = this.centers[high] as THREE.Vector3;
    frame.position.copy(c0).lerp(c1, fraction);
    const rotation = scratchQuaternion.copy(this.rotations[low] as THREE.Quaternion).slerp(this.rotations[high] as THREE.Quaternion, fraction);
    frame.tangent.set(1, 0, 0).applyQuaternion(rotation);
    frame.right.set(0, 1, 0).applyQuaternion(rotation);
    frame.down.set(0, 0, 1).applyQuaternion(rotation);
    return frame;
  }

  /** Centreline points in the horizontal plane (inertial x, y), every `step` samples, ends included. */
  planPoints(step: number): { x: number; y: number; station: number }[] {
    const points: { x: number; y: number; station: number }[] = [];
    const push = (index: number): void => {
      const c = this.centers[index] as THREE.Vector3;
      points.push({ x: c.x, y: c.y, station: this.stations[index] as number });
    };
    for (let index = 0; index < this.count; index += Math.max(1, step)) {
      push(index);
    }
    if ((this.count - 1) % Math.max(1, step) !== 0) {
      push(this.count - 1);
    }
    return points;
  }

  /** The station whose centreline point is nearest to an inertial position, searched near the last answer. */
  stationNearest(position: THREE.Vector3): number {
    const centers = this.centers;
    const distance = (index: number): number => {
      const c = centers[index] as THREE.Vector3;
      return (c.x - position.x) ** 2 + (c.y - position.y) ** 2;
    };
    let best = Math.min(Math.max(0, this.hint), centers.length - 1);
    let bestDistance = distance(best);
    // Walk downhill from the last answer; after a jump fall back to a full scan.
    for (let guard = 0; guard < centers.length; ++guard) {
      const previous = best > 0 ? distance(best - 1) : Infinity;
      const next = best + 1 < centers.length ? distance(best + 1) : Infinity;
      if (previous < bestDistance && previous <= next) {
        best -= 1;
        bestDistance = previous;
      } else if (next < bestDistance) {
        best += 1;
        bestDistance = next;
      } else {
        break;
      }
    }
    if (bestDistance > 4) {
      for (let index = 0; index < centers.length; ++index) {
        const d = distance(index);
        if (d < bestDistance) {
          best = index;
          bestDistance = d;
        }
      }
    }
    this.hint = best;
    // Refine along the neighbouring segment.
    const c = centers[best] as THREE.Vector3;
    const neighbour = best + 1 < centers.length ? best + 1 : best - 1;
    const n = centers[neighbour] as THREE.Vector3;
    const sx = n.x - c.x;
    const sy = n.y - c.y;
    const length2 = sx * sx + sy * sy;
    const t = length2 > 0 ? ((position.x - c.x) * sx + (position.y - c.y) * sy) / length2 : 0;
    const s0 = this.stations[best] as number;
    const s1 = this.stations[neighbour] as number;
    return s0 + t * (s1 - s0);
  }
}

const scratchQuaternion = new THREE.Quaternion();

function classifySections(stations: Float64Array, curvature: Float64Array): TrackSection[] {
  const count = stations.length;
  let kmax = 0;
  curvature.forEach((k) => {
    kmax = Math.max(kmax, Math.abs(k));
  });
  const kinds: SectionKind[] = new Array<SectionKind>(count).fill('tangent');
  const plateau = new Float64Array(count);
  if (kmax > 0) {
    const tangentLimit = 1e-4 * kmax;
    const flatLimit = 1e-6 * kmax;
    const k = (index: number): number => curvature[index] as number;
    const isTangent = (index: number): boolean => Math.abs(k(index)) <= tangentLimit;
    for (let index = 0; index < count; ++index) {
      kinds[index] = isTangent(index) ? 'tangent' : 'transition';
    }
    // Plateaus: runs of locally flat, non-tangent curvature at least 4 m long.
    let runStart = -1;
    const closeRun = (end: number): void => {
      if (runStart < 0) {
        return;
      }
      if ((stations[end] as number) - (stations[runStart] as number) >= 4) {
        const values = Array.from(curvature.subarray(runStart, end + 1)).sort((a, b) => a - b);
        const value = values[values.length >> 1] as number;
        let low = runStart;
        let high = end;
        const within = (index: number): boolean => Math.abs(k(index) - value) <= 1e-4 * Math.abs(value);
        while (low > 0 && within(low - 1)) {
          --low;
        }
        while (high < count - 1 && within(high + 1)) {
          ++high;
        }
        for (let index = low; index <= high; ++index) {
          kinds[index] = 'circular';
          plateau[index] = value;
        }
      }
      runStart = -1;
    };
    for (let index = 1; index < count - 1; ++index) {
      const flat = !isTangent(index) && Math.abs(k(index + 1) - k(index)) <= flatLimit && Math.abs(k(index) - k(index - 1)) <= flatLimit;
      if (flat && runStart < 0) {
        runStart = index;
      } else if (!flat && runStart >= 0) {
        closeRun(index - 1);
      }
    }
    closeRun(count - 2);
  }
  // Runs of one kind and, for circular samples, one plateau: curves of
  // different radius stay apart. A run shorter than 4 m between two runs of the
  // same key is a seam blip and is absorbed; so is a short run at either end.
  interface Run {
    start: number;
    end: number;
    kind: SectionKind;
    plateau: number;
  }
  const runs: Run[] = [];
  for (let index = 0; index < count; ++index) {
    const kind = kinds[index] as SectionKind;
    const value = plateau[index] as number;
    const previous = runs[runs.length - 1];
    if (previous !== undefined && previous.kind === kind && previous.plateau === value) {
      previous.end = index;
    } else {
      runs.push({ start: index, end: index, kind, plateau: value });
    }
  }
  const lengthOf = (run: Run): number => (stations[run.end] as number) - (stations[run.start] as number);
  const sameKey = (a: Run, b: Run): boolean => a.kind === b.kind && a.plateau === b.plateau;
  for (let changed = true; changed; ) {
    changed = false;
    for (let position = 1; position + 1 < runs.length; ++position) {
      const before = runs[position - 1] as Run;
      const run = runs[position] as Run;
      const after = runs[position + 1] as Run;
      if (lengthOf(run) < 4 && sameKey(before, after)) {
        before.end = after.end;
        runs.splice(position, 2);
        changed = true;
        break;
      }
    }
  }
  if (runs.length > 1 && lengthOf(runs[0] as Run) < 4) {
    (runs[1] as Run).start = (runs[0] as Run).start;
    runs.shift();
  }
  if (runs.length > 1 && lengthOf(runs[runs.length - 1] as Run) < 4) {
    (runs[runs.length - 2] as Run).end = (runs[runs.length - 1] as Run).end;
    runs.pop();
  }
  return runs.map((run, position) => {
    const next = runs[position + 1];
    const startStation = position === 0 ? (stations[run.start] as number) : 0.5 * ((stations[run.start] as number) + (stations[run.start - 1] as number));
    const endStation = next === undefined ? (stations[run.end] as number) : 0.5 * ((stations[run.end] as number) + (stations[run.end + 1] as number));
    const middle = Math.floor(0.5 * (run.start + run.end));
    const kMiddle = run.kind === 'circular' ? run.plateau : (curvature[middle] as number);
    return {
      kind: run.kind,
      startStation,
      endStation,
      radiusMeters: run.kind === 'circular' && run.plateau !== 0 ? 1 / Math.abs(run.plateau) : null,
      direction: run.kind === 'tangent' ? 0 : Math.sign(kMiddle),
    };
  });
}

function elementPointsOf(sections: TrackSection[]): ElementPoint[] {
  const points: ElementPoint[] = [];
  for (let index = 1; index < sections.length; ++index) {
    const before = (sections[index - 1] as TrackSection).kind;
    const after = (sections[index] as TrackSection).kind;
    const station = (sections[index] as TrackSection).startStation;
    let code: ElementPoint['code'] | null = null;
    if (before === 'tangent' && after === 'transition') code = 'TS';
    else if (before === 'transition' && after === 'circular') code = 'SC';
    else if (before === 'circular' && after === 'transition') code = 'CS';
    else if (before === 'transition' && after === 'tangent') code = 'ST';
    else if (before === 'tangent' && after === 'circular') code = 'TC';
    else if (before === 'circular' && after === 'tangent') code = 'CT';
    else if (before === 'circular' && after === 'circular') code = 'CC';
    if (code !== null) {
      points.push({ code, zh: elementNames[code], station });
    }
  }
  return points;
}

export const sectionNames: Record<SectionKind, { en: string; zh: string }> = {
  tangent: { en: 'Tangent track', zh: '直线' },
  transition: { en: 'Transition curve', zh: '缓和曲线' },
  circular: { en: 'Circular curve', zh: '圆曲线' },
};
