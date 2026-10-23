import type { TrackModel } from './track_model.ts';

export const trackSegmentLengthMeters = 60;
export type TrackDetail = 'detailed' | 'simplified';

export interface TrackSegment {
  id: number;
  start: number;
  end: number;
  includesEnd: boolean;
}

export function trackSegmentBounds(model: TrackModel): [number, number] {
  const first = Math.floor(model.firstStationMeters / trackSegmentLengthMeters);
  const last = Math.max(first, Math.ceil(model.lastStationMeters / trackSegmentLengthMeters) - 1);
  return [first, last];
}

export function trackSegmentAt(model: TrackModel, station: number): number {
  const [first, last] = trackSegmentBounds(model);
  return Math.max(first, Math.min(last, Math.floor(station / trackSegmentLengthMeters)));
}

export function trackSegment(model: TrackModel, id: number): TrackSegment {
  const [first, last] = trackSegmentBounds(model);
  if (!Number.isInteger(id) || id < first || id > last) throw new RangeError(`track segment ${id} is outside the table`);
  return { id, start: Math.max(model.firstStationMeters, id * trackSegmentLengthMeters),
    end: Math.min(model.lastStationMeters, (id + 1) * trackSegmentLengthMeters), includesEnd: id === last };
}

export function ownsStation(segment: TrackSegment, station: number): boolean {
  return station >= segment.start && (station < segment.end || (segment.includesEnd && station === segment.end));
}

/** Absolute integer grid: adjacent segments never duplicate sleepers or signs. */
export function ownedGridStations(segment: TrackSegment, spacing: number): number[] {
  const stations: number[] = [];
  for (let index = Math.floor(segment.start / spacing); ; ++index) {
    const station = index * spacing;
    if (station > segment.end || (station === segment.end && !segment.includesEnd)) break;
    if (ownsStation(segment, station)) stations.push(station);
  }
  return stations;
}

/** Surface samples include both ends, even for a very short clipped segment. */
export function segmentSurfaceStations(model: TrackModel, segment: TrackSegment, detail: TrackDetail): number[] {
  const stations = [segment.start];
  if (detail === 'detailed') {
    const begin = Math.floor(model.fractionalStationIndexOf(segment.start)) + 1;
    for (let index = begin; index < model.stationCount; ++index) {
      const station = model.stationsMeters[index] as number;
      if (station >= segment.end) break;
      stations.push(station);
    }
  } else {
    for (let index = Math.floor(segment.start / 3) + 1; index * 3 < segment.end; ++index) stations.push(index * 3);
  }
  if (segment.end > segment.start) stations.push(segment.end);
  return stations;
}
