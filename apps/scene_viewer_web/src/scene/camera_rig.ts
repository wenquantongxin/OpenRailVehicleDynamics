import * as THREE from 'three';
import type { OrbitControls } from 'three/addons/controls/OrbitControls.js';

import type { SceneRecord } from '../record/scene_record.ts';
import { fitDistanceMeters, type ViewportArea } from './camera_framing.ts';
import {
  anchorLocalFrame,
  extentCentre,
  extentCorners,
  extentHeightMeters,
  extentLengthMeters,
  measureDisplayExtent,
  type DisplayExtent,
  type LocalFrame,
} from './display_extents.ts';
import type { BuiltScene } from './scene_builder.ts';
import type { TrackModel } from './track_model.ts';
import type { VehicleDisplayBindings } from './vehicle_display_bindings.ts';

// Camera presets in four layers. Taste: a viewing direction, a field of view,
// a target stated as fractions of the framed extent and a named margin. The
// framed object: the whole vehicle, or one bogie's running gear. The target:
// the extent's centre plus that offset, never a body origin. The fit: the
// distance at which the extent's eight corners fall inside the picture left
// free by the cards, solved in camera_framing.ts. Presets are stated in the
// anchor's local frame (forward, left, up) whose heading is the recorded line
// at the anchor's station, so "front" stays in front through a curve and the
// view does not sway with the carbody.
//
// Anchors come from the display bindings, never from a body's name. A preset
// framing the vehicle falls back to the mean position of all bodies when no
// carbody is bound; the bogie presets need a bound bogie with measured parts,
// and the same rule decides the buttons, keys and deep links.

export type ViewPreset = 'overview' | 'bogie' | 'headon' | 'plan' | 'side';

export const viewPresets: ViewPreset[] = ['overview', 'bogie', 'headon', 'plan', 'side'];

type AnchorRole = 'carbody' | 'bogie';

/** What a preset must show: the vehicle, the first bound bogie, or that bogie when bound and the vehicle otherwise. */
type FitObject = 'vehicle' | 'bogie' | 'bogieElseVehicle';

interface PresetSpec {
  fit: FitObject;
  /** Direction from the target to the camera: forward, left, up (normalised in use). */
  direction: [number, number, number];
  fov: number;
  /** Margin kept between the framed corners and the free rectangle, as a fraction. */
  margin: number;
  /** Look-at point in the anchor frame, from the framed extent. */
  target: (extent: DisplayExtent) => [number, number, number];
}

const presets: Record<ViewPreset, PresetSpec> = {
  overview: {
    fit: 'vehicle',
    direction: [0.6, 0.74, 0.21],
    fov: 30,
    margin: 0.08,
    target: (extent) => {
      const [forward, left] = extentCentre(extent);
      return [forward + 0.055 * extentLengthMeters(extent), left, extent.minUp + 0.32 * extentHeightMeters(extent)];
    },
  },
  bogie: { fit: 'bogie', direction: [0.6, 0.72, 0.3], fov: 30, margin: 0.12, target: (extent) => extentCentre(extent) },
  // Head-on shows the running gear: it frames the first bogie from ahead of its front face.
  headon: {
    fit: 'bogieElseVehicle',
    direction: [1, 0, 0.055],
    fov: 13,
    margin: 0.15,
    target: (extent) => {
      const [, left] = extentCentre(extent);
      return [extent.maxForward, left, extent.minUp + 0.17 * extentHeightMeters(extent)];
    },
  },
  plan: { fit: 'vehicle', direction: [0, -0.0008, 1], fov: 32, margin: 0.06, target: (extent) => extentCentre(extent) },
  side: {
    fit: 'vehicle',
    direction: [0, 1, 0.16],
    fov: 30,
    margin: 0.06,
    target: (extent) => {
      const [forward, left] = extentCentre(extent);
      return [forward, left, extent.minUp + 0.4 * extentHeightMeters(extent)];
    },
  },
};

export interface CameraAnchors {
  root: THREE.Object3D;
  carbody: THREE.Object3D | null;
  bogie: THREE.Object3D | null;
  /** Every body object; their mean position stands in for an unbound carbody. */
  bodies: THREE.Object3D[];
}

/** Extents measured at the first displayed pose, each in its own anchor frame. */
export interface FramedExtents {
  vehicle: DisplayExtent | null;
  bogie: DisplayExtent | null;
}

export function cameraAnchorsFor(scene: BuiltScene, bindings: VehicleDisplayBindings): CameraAnchors {
  const firstBogie = bindings.bogies[0];
  return {
    root: scene.root,
    carbody: bindings.carbody === null ? null : (scene.bodyObjects.get(bindings.carbody.bodyName) ?? null),
    bogie: firstBogie === undefined ? null : (scene.bodyObjects.get(firstBogie.bodyName) ?? null),
    bodies: [...scene.bodyObjects.values()],
  };
}

const scratchPosition = new THREE.Vector3();

/** World position of an anchor role, or null when the role has no body and no stand-in. */
export function anchorOriginWorld(role: AnchorRole, anchors: CameraAnchors): THREE.Vector3 | null {
  if (role === 'bogie') {
    return anchors.bogie === null ? null : anchors.bogie.getWorldPosition(new THREE.Vector3());
  }
  if (anchors.carbody !== null) {
    return anchors.carbody.getWorldPosition(new THREE.Vector3());
  }
  if (anchors.bodies.length === 0) {
    return null;
  }
  const mean = new THREE.Vector3();
  for (const body of anchors.bodies) {
    mean.add(body.getWorldPosition(scratchPosition));
  }
  return mean.multiplyScalar(1 / anchors.bodies.length);
}

/** The +x axis of an anchor body in the world, for a scene without a line; +x forward for a stand-in. */
function anchorForwardWorld(role: AnchorRole, anchors: CameraAnchors): THREE.Vector3 {
  const body = role === 'bogie' ? anchors.bogie : anchors.carbody;
  const forward = new THREE.Vector3(1, 0, 0);
  return body === null ? forward : forward.applyQuaternion(body.getWorldQuaternion(new THREE.Quaternion()));
}

/** The framed extent and anchor role a preset resolves to, or null when it cannot be framed. */
export function presetFitObject(preset: ViewPreset, extents: FramedExtents, bogieBound: boolean): { role: AnchorRole; extent: DisplayExtent } | null {
  const fit = presets[preset].fit;
  if ((fit === 'bogie' || fit === 'bogieElseVehicle') && bogieBound && extents.bogie !== null) {
    return { role: 'bogie', extent: extents.bogie };
  }
  if (fit !== 'bogie' && extents.vehicle !== null) {
    return { role: 'carbody', extent: extents.vehicle };
  }
  return null;
}

/** One rule for buttons, keys, deep links and the rig itself. */
export function presetAvailableFor(preset: ViewPreset, extents: FramedExtents, bogieBound: boolean): boolean {
  return presetFitObject(preset, extents, bogieBound) !== null;
}

/**
 * Measures the vehicle and the first bound bogie at the scene's current pose,
 * each in the local frame of its anchor. Body matrices must be up to date.
 */
export function measureFramedExtents(record: SceneRecord, scene: BuiltScene, bindings: VehicleDisplayBindings): FramedExtents {
  const anchors = cameraAnchorsFor(scene, bindings);
  const vehicleOrigin = anchorOriginWorld('carbody', anchors);
  const vehicle =
    vehicleOrigin === null
      ? null
      : measureDisplayExtent(scene.parts, anchorLocalFrame(vehicleOrigin, scene.track, scene.root, anchorForwardWorld('carbody', anchors)), null);
  const firstBogie = bindings.bogies[0];
  const bogieOrigin = anchorOriginWorld('bogie', anchors);
  const bogie =
    firstBogie === undefined || bogieOrigin === null
      ? null
      : measureDisplayExtent(
          scene.parts,
          anchorLocalFrame(bogieOrigin, scene.track, scene.root, anchorForwardWorld('bogie', anchors)),
          new Set(firstBogie.memberBodyIndices.map((bodyIndex) => record.bodies[bodyIndex]?.name ?? '')),
        );
  return { vehicle, bogie };
}

/** The taste of a preset, for checks that project its framing: direction, field of view and margin. */
export function presetTaste(preset: ViewPreset): { direction: [number, number, number]; fov: number; margin: number } {
  const spec = presets[preset];
  return { direction: [...spec.direction], fov: spec.fov, margin: spec.margin };
}

/** Target and camera in the anchor frame; the preset's own framing rule. */
export function presetDestination(preset: ViewPreset, extent: DisplayExtent, viewport: ViewportArea): { camera: THREE.Vector3; target: THREE.Vector3 } {
  const spec = presets[preset];
  const target = spec.target(extent);
  const distance = fitDistanceMeters(extentCorners(extent), target, spec.direction, spec.fov, viewport, spec.margin);
  const direction = new THREE.Vector3(...spec.direction).normalize();
  return { camera: new THREE.Vector3(...target).addScaledVector(direction, distance), target: new THREE.Vector3(...target) };
}

function sameViewport(a: ViewportArea | null, b: ViewportArea): boolean {
  return (
    a !== null &&
    a.width === b.width &&
    a.height === b.height &&
    a.free.x0 === b.free.x0 &&
    a.free.y0 === b.free.y0 &&
    a.free.x1 === b.free.x1 &&
    a.free.y1 === b.free.y1
  );
}

function copyViewport(viewport: ViewportArea): ViewportArea {
  return { width: viewport.width, height: viewport.height, free: { ...viewport.free } };
}

function easeInOutCubic(s: number): number {
  return s < 0.5 ? 4 * s * s * s : 1 - (-2 * s + 2) ** 3 / 2;
}

export class CameraRig {
  follow = true;
  private preset: ViewPreset = 'overview';
  private tween: {
    startCamera: THREE.Vector3;
    startTarget: THREE.Vector3;
    startFov: number;
    elapsed: number;
    duration: number;
  } | null = null;
  private lastAnchorFrame: LocalFrame | null = null;
  /** The picture the current preset was fitted to; a change refits unless the user took over. */
  private fittedViewport: ViewportArea | null = null;
  /** Set when the user orbits or zooms; an automatic refit never overrides that view. */
  private userAdjusted = false;
  private readonly scratch = new THREE.Vector3();

  private readonly camera: THREE.PerspectiveCamera;
  private readonly controls: OrbitControls;
  private readonly anchors: CameraAnchors;
  private readonly track: TrackModel | null;
  private readonly extents: FramedExtents;

  constructor(camera: THREE.PerspectiveCamera, controls: OrbitControls, anchors: CameraAnchors, track: TrackModel | null, extents: FramedExtents) {
    this.camera = camera;
    this.controls = controls;
    this.anchors = anchors;
    this.track = track;
    this.extents = extents;
    controls.addEventListener('start', () => {
      this.tween = null;
      this.userAdjusted = true;
    });
  }

  get currentPreset(): ViewPreset {
    return this.preset;
  }

  presetAvailable(preset: ViewPreset): boolean {
    return presetAvailableFor(preset, this.extents, this.anchors.bogie !== null);
  }

  /** The anchor's world position and the local frame there, or null when the role is not bound. */
  private anchorFrameFor(role: AnchorRole): LocalFrame | null {
    const origin = anchorOriginWorld(role, this.anchors);
    return origin === null ? null : anchorLocalFrame(origin, this.track, this.anchors.root, anchorForwardWorld(role, this.anchors));
  }

  private toWorld(anchorFrame: LocalFrame, local: THREE.Vector3, out: THREE.Vector3): THREE.Vector3 {
    return out
      .copy(anchorFrame.origin)
      .addScaledVector(anchorFrame.forward, local.x)
      .addScaledVector(anchorFrame.left, local.y)
      .addScaledVector(anchorFrame.up, local.z);
  }

  private toLocal(anchorFrame: LocalFrame, world: THREE.Vector3, out: THREE.Vector3): THREE.Vector3 {
    this.scratch.copy(world).sub(anchorFrame.origin);
    return out.set(this.scratch.dot(anchorFrame.forward), this.scratch.dot(anchorFrame.left), this.scratch.dot(anchorFrame.up));
  }

  private place(preset: ViewPreset, extent: DisplayExtent, anchorFrame: LocalFrame, viewport: ViewportArea): void {
    const destination = presetDestination(preset, extent, viewport);
    this.toWorld(anchorFrame, destination.camera, this.camera.position);
    this.toWorld(anchorFrame, destination.target, this.controls.target);
    this.camera.fov = presets[preset].fov;
    this.camera.updateProjectionMatrix();
    this.tween = null;
    this.fittedViewport = copyViewport(viewport);
  }

  /** Moves to a preset; returns false and changes nothing when it cannot be framed. */
  applyPreset(preset: ViewPreset, viewport: ViewportArea, immediate: boolean): boolean {
    const fit = presetFitObject(preset, this.extents, this.anchors.bogie !== null);
    if (fit === null) {
      return false;
    }
    const anchorFrame = this.anchorFrameFor(fit.role);
    if (anchorFrame === null) {
      return false;
    }
    this.preset = preset;
    this.userAdjusted = false;
    if (immediate) {
      this.place(preset, fit.extent, anchorFrame, viewport);
    } else {
      this.tween = {
        startCamera: this.toLocal(anchorFrame, this.camera.position, new THREE.Vector3()),
        startTarget: this.toLocal(anchorFrame, this.controls.target, new THREE.Vector3()),
        startFov: this.camera.fov,
        elapsed: 0,
        duration: 0.9,
      };
      this.fittedViewport = null;
    }
    this.lastAnchorFrame = anchorFrame;
    this.controls.update();
    return true;
  }

  /** Refits the current preset to a changed picture, unless a tween runs or the user has taken over. */
  refit(viewport: ViewportArea): void {
    if (this.userAdjusted || this.tween !== null || sameViewport(this.fittedViewport, viewport)) {
      return;
    }
    const fit = presetFitObject(this.preset, this.extents, this.anchors.bogie !== null);
    const anchorFrame = fit === null ? null : this.anchorFrameFor(fit.role);
    if (fit === null || anchorFrame === null) {
      return;
    }
    this.place(this.preset, fit.extent, anchorFrame, viewport);
    this.lastAnchorFrame = anchorFrame;
    this.controls.update();
  }

  /** Called once per rendered frame after body poses are written. */
  update(wallDeltaSeconds: number, viewport: ViewportArea): void {
    const fit = presetFitObject(this.preset, this.extents, this.anchors.bogie !== null);
    const anchorFrame = fit === null ? null : this.anchorFrameFor(fit.role);
    if (fit === null || anchorFrame === null) {
      return;
    }
    if (this.tween !== null) {
      this.tween.elapsed += wallDeltaSeconds;
      const s = easeInOutCubic(Math.min(1, this.tween.elapsed / this.tween.duration));
      const destination = presetDestination(this.preset, fit.extent, viewport);
      const cameraLocal = this.tween.startCamera.clone().lerp(destination.camera, s);
      const targetLocal = this.tween.startTarget.clone().lerp(destination.target, s);
      this.toWorld(anchorFrame, cameraLocal, this.camera.position);
      this.toWorld(anchorFrame, targetLocal, this.controls.target);
      this.camera.fov = THREE.MathUtils.lerp(this.tween.startFov, presets[this.preset].fov, s);
      this.camera.updateProjectionMatrix();
      if (s >= 1) {
        this.tween = null;
        this.fittedViewport = copyViewport(viewport);
      }
    } else if (this.follow && this.lastAnchorFrame !== null) {
      // Carry the camera with the anchor: same offset, turned with the heading.
      const cameraLocal = this.toLocal(this.lastAnchorFrame, this.camera.position, new THREE.Vector3());
      const targetLocal = this.toLocal(this.lastAnchorFrame, this.controls.target, new THREE.Vector3());
      this.toWorld(anchorFrame, cameraLocal, this.camera.position);
      this.toWorld(anchorFrame, targetLocal, this.controls.target);
    }
    this.lastAnchorFrame = anchorFrame;
    this.controls.update();
  }
}
