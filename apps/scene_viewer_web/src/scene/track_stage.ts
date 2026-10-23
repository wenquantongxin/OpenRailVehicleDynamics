import * as THREE from 'three';

import type { ScenePalette } from './materials.ts';
import { buildGround, buildTrackSegment, disposeTrackSegment, moveGround } from './track_builder.ts';
import type { TrackModel } from './track_model.ts';
import { trackSegment, trackSegmentAt, trackSegmentBounds, type TrackDetail } from './track_segments.ts';

/** A bounded geometry window over the recorded line, independent of simulation state. */
export class TrackStage {
  readonly group = new THREE.Group();
  readonly ground: THREE.Mesh;
  private readonly model: TrackModel;
  private readonly palette: ScenePalette;
  private readonly segments = new Map<number, { detail: TrackDetail; group: THREE.Group }>();
  private focusSegment: number | null = null;

  constructor(model: TrackModel, palette: ScenePalette) {
    this.model = model;
    this.palette = palette;
    this.group.name = 'track';
    this.ground = buildGround(model, palette);
  }

  update(stationMeters: number, groundPosition?: THREE.Vector3): void {
    const focus = groundPosition ?? this.model.trackFrameAtStation(stationMeters).position;
    moveGround(this.ground, focus.x, focus.y);
    const id = trackSegmentAt(this.model, stationMeters);
    if (id === this.focusSegment) return;
    const [first, last] = trackSegmentBounds(this.model);
    const low = Math.max(first, id - 8);
    const high = Math.min(last, id + 8);
    for (const [key, segment] of this.segments) {
      const detail: TrackDetail = Math.abs(key - id) <= 2 ? 'detailed' : 'simplified';
      if (key < low || key > high || segment.detail !== detail) {
        disposeTrackSegment(segment.group);
        this.segments.delete(key);
      }
    }
    for (let key = low; key <= high; ++key) {
      if (this.segments.has(key)) continue;
      const detail: TrackDetail = Math.abs(key - id) <= 2 ? 'detailed' : 'simplified';
      const group = buildTrackSegment(this.model, this.palette, trackSegment(this.model, key), detail);
      this.group.add(group);
      this.segments.set(key, { detail, group });
    }
    this.focusSegment = id;
  }

  dispose(): void {
    for (const segment of this.segments.values()) disposeTrackSegment(segment.group);
    this.segments.clear();
    this.focusSegment = null;
    this.ground.geometry.dispose();
    this.ground.removeFromParent();
    this.group.removeFromParent();
  }
}
