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
// plateau at least 4 m long measured between the plateau's real edges (each
// plateau keeps its own radius), transition elsewhere. A run shorter than 4 m
// between two runs of the same kind and radius is absorbed. On the bundled
// R300 line this places the element points within 0.3 m of the line
// definition; the record carries no segment table. The classification is a
// display description derived from finite samples, not a second authority on
// the line.
//
// Rail positions come from the recorded rail datums, expressed per station in
// Track-T about the centreline, so a different datum spacing is drawn as
// recorded.

export type SectionKind = 'tangent' | 'transition' | 'circular';

export interface TrackSection {
  kind: SectionKind;
  startStationMeters: number;
  endStationMeters: number;
  /** Plateau radius of a circular section; null otherwise. */
  radiusMeters: number | null;
  /** +1 turns right (towards +y), -1 turns left, 0 straight. */
  direction: number;
}

export interface ElementPoint {
  code: 'TS' | 'SC' | 'CS' | 'ST' | 'TC' | 'CT' | 'CC';
  zh: string;
  stationMeters: number;
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

/** The Track-T frame at one station: origin and axes expressed in I. */
export interface TrackFrame {
  position: THREE.Vector3;
  /** Along track, right, down: the Track-T axes expressed in I. */
  tangent: THREE.Vector3;
  right: THREE.Vector3;
  down: THREE.Vector3;
}

export class TrackModel {
  readonly stationsMeters: Float64Array;
  readonly curvatureRadiansPerMeter: Float64Array;
  readonly superelevationMeters: Float64Array;
  readonly sections: TrackSection[];
  readonly elementPoints: ElementPoint[];
  /** Recorded rail datums in Track-T about the centreline: lateral offset (right positive) and depth offset (down positive), metres. */
  readonly leftRailLateralOffsetsMeters: Float64Array;
  readonly leftRailDepthOffsetsMeters: Float64Array;
  readonly rightRailLateralOffsetsMeters: Float64Array;
  readonly rightRailDepthOffsetsMeters: Float64Array;
  /** The lowest centreline point of the table as inertial z, which points down, so it is the largest z. */
  readonly lowestCentrelineInertialZMeters: number;
  /** Median lateral distance between the two recorded rail datums; not the gauge, which is measured between the rail heads. */
  readonly medianRailDatumSpacingMeters: number;
  private readonly centres: THREE.Vector3[];
  private readonly rotations: THREE.Quaternion[];
  private nearestHint = 0;

  constructor(table: TrackTable) {
    const count = table.stationsMeters.length;
    this.stationsMeters = Float64Array.from(table.stationsMeters);
    this.curvatureRadiansPerMeter = Float64Array.from(table.curvatureRadiansPerMeter);
    this.superelevationMeters = Float64Array.from(table.superelevationMeters);
    this.centres = table.centerlineInInertialMeters.map((row) => new THREE.Vector3(row[0] ?? 0, row[1] ?? 0, row[2] ?? 0));
    this.rotations = table.rotationInertialFromTrackWxyz.map((row) =>
      new THREE.Quaternion(row[1] ?? 0, row[2] ?? 0, row[3] ?? 0, row[0] ?? 1).normalize(),
    );
    if (
      this.curvatureRadiansPerMeter.length !== count ||
      this.rotations.length !== count ||
      this.superelevationMeters.length !== count ||
      table.leftRailDatumInInertialMeters.length !== count ||
      table.rightRailDatumInInertialMeters.length !== count
    ) {
      throw new Error('track table: curvature, cant, rotation and rail columns disagree with the stations');
    }
    this.leftRailLateralOffsetsMeters = new Float64Array(count);
    this.leftRailDepthOffsetsMeters = new Float64Array(count);
    this.rightRailLateralOffsetsMeters = new Float64Array(count);
    this.rightRailDepthOffsetsMeters = new Float64Array(count);
    const offset = new THREE.Vector3();
    const right = new THREE.Vector3();
    const down = new THREE.Vector3();
    let lowest = -Infinity;
    const spacings = new Float64Array(count);
    for (let stationIndex = 0; stationIndex < count; ++stationIndex) {
      const centre = this.centres[stationIndex] as THREE.Vector3;
      const rotation = this.rotations[stationIndex] as THREE.Quaternion;
      right.set(0, 1, 0).applyQuaternion(rotation);
      down.set(0, 0, 1).applyQuaternion(rotation);
      const left = table.leftRailDatumInInertialMeters[stationIndex] ?? [];
      offset.set(left[0] ?? 0, left[1] ?? 0, left[2] ?? 0).sub(centre);
      this.leftRailLateralOffsetsMeters[stationIndex] = offset.dot(right);
      this.leftRailDepthOffsetsMeters[stationIndex] = offset.dot(down);
      const rightRail = table.rightRailDatumInInertialMeters[stationIndex] ?? [];
      offset.set(rightRail[0] ?? 0, rightRail[1] ?? 0, rightRail[2] ?? 0).sub(centre);
      this.rightRailLateralOffsetsMeters[stationIndex] = offset.dot(right);
      this.rightRailDepthOffsetsMeters[stationIndex] = offset.dot(down);
      spacings[stationIndex] =
        (this.rightRailLateralOffsetsMeters[stationIndex] as number) - (this.leftRailLateralOffsetsMeters[stationIndex] as number);
      lowest = Math.max(lowest, centre.z);
    }
    this.lowestCentrelineInertialZMeters = count > 0 ? lowest : 0;
    spacings.sort();
    this.medianRailDatumSpacingMeters = count > 0 ? (spacings[count >> 1] as number) : 1.435;
    this.sections = classifySections(this.stationsMeters, this.curvatureRadiansPerMeter);
    this.elementPoints = elementPointsOf(this.sections);
  }

  get stationCount(): number {
    return this.stationsMeters.length;
  }

  get firstStationMeters(): number {
    return this.stationsMeters[0] as number;
  }

  get lastStationMeters(): number {
    return this.stationsMeters[this.stationsMeters.length - 1] as number;
  }

  /** Fractional table index of a station, clamped to the table. */
  fractionalStationIndexOf(stationMeters: number): number {
    const s = this.stationsMeters;
    const last = s.length - 1;
    if (stationMeters <= (s[0] as number)) {
      return 0;
    }
    if (stationMeters >= (s[last] as number)) {
      return last;
    }
    let low = 0;
    let high = last;
    while (high - low > 1) {
      const middle = (low + high) >> 1;
      if ((s[middle] as number) <= stationMeters) {
        low = middle;
      } else {
        high = middle;
      }
    }
    const s0 = s[low] as number;
    const s1 = s[high] as number;
    return low + (stationMeters - s0) / (s1 - s0);
  }

  private interpolate(values: Float64Array, stationMeters: number): number {
    const index = this.fractionalStationIndexOf(stationMeters);
    const low = Math.floor(index);
    const high = Math.min(values.length - 1, low + 1);
    const fraction = index - low;
    return (values[low] as number) * (1 - fraction) + (values[high] as number) * fraction;
  }

  /** Recorded rail datum of one side at a station, in Track-T about the centreline: [lateral, depth] metres. */
  railDatumOffsetAtStation(stationMeters: number, side: 'left' | 'right'): [number, number] {
    const lateral = side === 'left' ? this.leftRailLateralOffsetsMeters : this.rightRailLateralOffsetsMeters;
    const depth = side === 'left' ? this.leftRailDepthOffsetsMeters : this.rightRailDepthOffsetsMeters;
    return [this.interpolate(lateral, stationMeters), this.interpolate(depth, stationMeters)];
  }

  curvatureAtStation(stationMeters: number): number {
    return this.interpolate(this.curvatureRadiansPerMeter, stationMeters);
  }

  superelevationAtStation(stationMeters: number): number {
    return this.interpolate(this.superelevationMeters, stationMeters);
  }

  sectionAtStation(stationMeters: number): TrackSection | null {
    for (const section of this.sections) {
      if (stationMeters >= section.startStationMeters && stationMeters <= section.endStationMeters) {
        return section;
      }
    }
    return null;
  }

  /** Origin and Track-T axes at a station, interpolated between samples. */
  trackFrameAtStation(stationMeters: number, out?: TrackFrame): TrackFrame {
    const trackFrame = out ?? {
      position: new THREE.Vector3(),
      tangent: new THREE.Vector3(),
      right: new THREE.Vector3(),
      down: new THREE.Vector3(),
    };
    const index = this.fractionalStationIndexOf(stationMeters);
    const low = Math.floor(index);
    const high = Math.min(this.stationCount - 1, low + 1);
    const fraction = index - low;
    const c0 = this.centres[low] as THREE.Vector3;
    const c1 = this.centres[high] as THREE.Vector3;
    trackFrame.position.copy(c0).lerp(c1, fraction);
    const rotation = scratchQuaternion.copy(this.rotations[low] as THREE.Quaternion).slerp(this.rotations[high] as THREE.Quaternion, fraction);
    trackFrame.tangent.set(1, 0, 0).applyQuaternion(rotation);
    trackFrame.right.set(0, 1, 0).applyQuaternion(rotation);
    trackFrame.down.set(0, 0, 1).applyQuaternion(rotation);
    return trackFrame;
  }

  /** Centreline points in the horizontal plane (inertial x, y), every `stationStep` samples, ends included. */
  planPointsEvery(stationStep: number): { x: number; y: number; stationMeters: number }[] {
    const points: { x: number; y: number; stationMeters: number }[] = [];
    const push = (stationIndex: number): void => {
      const c = this.centres[stationIndex] as THREE.Vector3;
      points.push({ x: c.x, y: c.y, stationMeters: this.stationsMeters[stationIndex] as number });
    };
    const step = Math.max(1, stationStep);
    for (let stationIndex = 0; stationIndex < this.stationCount; stationIndex += step) {
      push(stationIndex);
    }
    if ((this.stationCount - 1) % step !== 0) {
      push(this.stationCount - 1);
    }
    return points;
  }

  /** The station whose centreline point is nearest to an inertial position, searched near the last answer. */
  stationNearestToInertialPosition(position: THREE.Vector3): number {
    const centres = this.centres;
    const distance = (stationIndex: number): number => {
      const c = centres[stationIndex] as THREE.Vector3;
      return (c.x - position.x) ** 2 + (c.y - position.y) ** 2;
    };
    let best = Math.min(Math.max(0, this.nearestHint), centres.length - 1);
    let bestDistance = distance(best);
    // Walk downhill from the last answer; after a jump fall back to a full scan.
    for (let guard = 0; guard < centres.length; ++guard) {
      const previous = best > 0 ? distance(best - 1) : Infinity;
      const next = best + 1 < centres.length ? distance(best + 1) : Infinity;
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
      for (let stationIndex = 0; stationIndex < centres.length; ++stationIndex) {
        const d = distance(stationIndex);
        if (d < bestDistance) {
          best = stationIndex;
          bestDistance = d;
        }
      }
    }
    this.nearestHint = best;
    // Refine along the neighbouring segment.
    const c = centres[best] as THREE.Vector3;
    const neighbour = best + 1 < centres.length ? best + 1 : best - 1;
    const n = centres[neighbour] as THREE.Vector3;
    const sx = n.x - c.x;
    const sy = n.y - c.y;
    const length2 = sx * sx + sy * sy;
    const t = length2 > 0 ? ((position.x - c.x) * sx + (position.y - c.y) * sy) / length2 : 0;
    const s0 = this.stationsMeters[best] as number;
    const s1 = this.stationsMeters[neighbour] as number;
    return s0 + t * (s1 - s0);
  }
}

const scratchQuaternion = new THREE.Quaternion();

function classifySections(stationsMeters: Float64Array, curvature: Float64Array): TrackSection[] {
  const count = stationsMeters.length;
  let kmax = 0;
  curvature.forEach((k) => {
    kmax = Math.max(kmax, Math.abs(k));
  });
  const kinds: SectionKind[] = new Array<SectionKind>(count).fill('tangent');
  const plateau = new Float64Array(count);
  if (kmax > 0) {
    const tangentLimit = 1e-4 * kmax;
    const flatLimit = 1e-6 * kmax;
    const k = (stationIndex: number): number => curvature[stationIndex] as number;
    const isTangent = (stationIndex: number): boolean => Math.abs(k(stationIndex)) <= tangentLimit;
    for (let stationIndex = 0; stationIndex < count; ++stationIndex) {
      kinds[stationIndex] = isTangent(stationIndex) ? 'tangent' : 'transition';
    }
    // Plateaus: runs of locally flat, non-tangent curvature. A run holds the
    // plateau's interior samples only, so it is first extended to every
    // neighbouring sample on the same value and the 4 m length is judged
    // between the plateau's real edges.
    let runStart = -1;
    const closeRun = (end: number): void => {
      if (runStart < 0) {
        return;
      }
      const values = Array.from(curvature.subarray(runStart, end + 1)).sort((a, b) => a - b);
      const value = values[values.length >> 1] as number;
      let low = runStart;
      let high = end;
      const within = (stationIndex: number): boolean => Math.abs(k(stationIndex) - value) <= 1e-4 * Math.abs(value);
      while (low > 0 && within(low - 1)) {
        --low;
      }
      while (high < count - 1 && within(high + 1)) {
        ++high;
      }
      if ((stationsMeters[high] as number) - (stationsMeters[low] as number) >= 4) {
        for (let stationIndex = low; stationIndex <= high; ++stationIndex) {
          kinds[stationIndex] = 'circular';
          plateau[stationIndex] = value;
        }
      }
      runStart = -1;
    };
    for (let stationIndex = 1; stationIndex < count - 1; ++stationIndex) {
      const flat =
        !isTangent(stationIndex) &&
        Math.abs(k(stationIndex + 1) - k(stationIndex)) <= flatLimit &&
        Math.abs(k(stationIndex) - k(stationIndex - 1)) <= flatLimit;
      if (flat && runStart < 0) {
        runStart = stationIndex;
      } else if (!flat && runStart >= 0) {
        closeRun(stationIndex - 1);
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
  for (let stationIndex = 0; stationIndex < count; ++stationIndex) {
    const kind = kinds[stationIndex] as SectionKind;
    const value = plateau[stationIndex] as number;
    const previous = runs[runs.length - 1];
    if (previous !== undefined && previous.kind === kind && previous.plateau === value) {
      previous.end = stationIndex;
    } else {
      runs.push({ start: stationIndex, end: stationIndex, kind, plateau: value });
    }
  }
  const lengthOf = (run: Run): number => (stationsMeters[run.end] as number) - (stationsMeters[run.start] as number);
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
    const startStationMeters =
      position === 0 ? (stationsMeters[run.start] as number) : 0.5 * ((stationsMeters[run.start] as number) + (stationsMeters[run.start - 1] as number));
    const endStationMeters =
      next === undefined ? (stationsMeters[run.end] as number) : 0.5 * ((stationsMeters[run.end] as number) + (stationsMeters[run.end + 1] as number));
    const middle = Math.floor(0.5 * (run.start + run.end));
    const kMiddle = run.kind === 'circular' ? run.plateau : (curvature[middle] as number);
    return {
      kind: run.kind,
      startStationMeters,
      endStationMeters,
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
    const stationMeters = (sections[index] as TrackSection).startStationMeters;
    let code: ElementPoint['code'] | null = null;
    if (before === 'tangent' && after === 'transition') code = 'TS';
    else if (before === 'transition' && after === 'circular') code = 'SC';
    else if (before === 'circular' && after === 'transition') code = 'CS';
    else if (before === 'transition' && after === 'tangent') code = 'ST';
    else if (before === 'tangent' && after === 'circular') code = 'TC';
    else if (before === 'circular' && after === 'tangent') code = 'CT';
    else if (before === 'circular' && after === 'circular') code = 'CC';
    if (code !== null) {
      points.push({ code, zh: elementNames[code], stationMeters });
    }
  }
  return points;
}

export const sectionNames: Record<SectionKind, { en: string; zh: string }> = {
  tangent: { en: 'Tangent track', zh: '直线' },
  transition: { en: 'Transition curve', zh: '缓和曲线' },
  circular: { en: 'Circular curve', zh: '圆曲线' },
};
