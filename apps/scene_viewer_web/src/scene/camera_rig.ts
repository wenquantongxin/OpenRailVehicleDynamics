import * as THREE from 'three';
import type { OrbitControls } from 'three/addons/controls/OrbitControls.js';

import type { TrackModel } from './track_model.ts';

// Camera presets stated in the line's local frame at the anchor (forward,
// left, up), so "front" stays in front of the vehicle through a curve. The
// heading comes from the recorded track frame at the anchor's station, not
// from the body's own yaw, so the view does not sway with the carbody.
//
// Anchors come from the display bindings, never from a body's name. A preset
// anchored to the carbody falls back to the mean position of all bodies when
// no carbody is bound, which assumes nothing about any one body; the bogie
// preset needs a bound bogie and is unavailable otherwise.

export type ViewPreset = 'overview' | 'bogie' | 'headon' | 'plan' | 'side';

export const viewPresets: ViewPreset[] = ['overview', 'bogie', 'headon', 'plan', 'side'];

type AnchorRole = 'carbody' | 'bogie';

interface PresetSpec {
  anchor: AnchorRole;
  /** Look-at point relative to the anchor: forward, left, up (metres). */
  target: [number, number, number];
  /** Direction from the target to the camera: forward, left, up (normalised here). */
  direction: [number, number, number];
  /** Horizontal extent in metres that must fit between the card columns. */
  fitWidth: number;
  fov: number;
}

const presets: Record<ViewPreset, PresetSpec> = {
  overview: { anchor: 'carbody', target: [1.2, 0, 1.2], direction: [0.6, 0.74, 0.21], fitWidth: 19.5, fov: 30 },
  bogie: { anchor: 'bogie', target: [0.05, 0, 0.08], direction: [0.6, 0.72, 0.3], fitWidth: 6.6, fov: 30 },
  headon: { anchor: 'carbody', target: [9.4, 0, 0.62], direction: [1, 0, 0.055], fitWidth: 4.2, fov: 13 },
  plan: { anchor: 'carbody', target: [0, 0, 0.6], direction: [0, -0.0008, 1], fitWidth: 24, fov: 32 },
  side: { anchor: 'carbody', target: [0, 0, 1.5], direction: [0, 1, 0.16], fitWidth: 23.5, fov: 30 },
};

/** Whether a preset needs a bound bogie; every other preset works with or without a bound carbody. */
export function presetRequiresBogie(preset: ViewPreset): boolean {
  return presets[preset].anchor === 'bogie';
}

const up = new THREE.Vector3(0, 1, 0);

export interface CameraAnchors {
  root: THREE.Object3D;
  carbody: THREE.Object3D | null;
  bogie: THREE.Object3D | null;
  /** Every body object; their mean position stands in for an unbound carbody. */
  bodies: THREE.Object3D[];
}

interface AnchorFrame {
  origin: THREE.Vector3;
  forward: THREE.Vector3;
  left: THREE.Vector3;
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
  private lastAnchorFrame: AnchorFrame | null = null;
  private readonly scratch = new THREE.Vector3();
  private readonly inertial = new THREE.Vector3();

  private readonly camera: THREE.PerspectiveCamera;
  private readonly controls: OrbitControls;
  private readonly anchors: CameraAnchors;
  private readonly track: TrackModel | null;

  constructor(camera: THREE.PerspectiveCamera, controls: OrbitControls, anchors: CameraAnchors, track: TrackModel | null) {
    this.camera = camera;
    this.controls = controls;
    this.anchors = anchors;
    this.track = track;
    controls.addEventListener('start', () => {
      this.tween = null;
    });
  }

  get currentPreset(): ViewPreset {
    return this.preset;
  }

  presetAvailable(preset: ViewPreset): boolean {
    return !presetRequiresBogie(preset) || this.anchors.bogie !== null;
  }

  /** The anchor's world position for a role, or null when the role is not bound and has no stand-in. */
  private anchorOrigin(role: AnchorRole): THREE.Vector3 | null {
    if (role === 'bogie') {
      return this.anchors.bogie === null ? null : this.anchors.bogie.getWorldPosition(new THREE.Vector3());
    }
    if (this.anchors.carbody !== null) {
      return this.anchors.carbody.getWorldPosition(new THREE.Vector3());
    }
    if (this.anchors.bodies.length === 0) {
      return null;
    }
    const mean = new THREE.Vector3();
    for (const body of this.anchors.bodies) {
      mean.add(body.getWorldPosition(this.scratch));
    }
    return mean.multiplyScalar(1 / this.anchors.bodies.length);
  }

  /** The anchor's world position and the horizontal track heading there. */
  private anchorFrameFor(role: AnchorRole): AnchorFrame | null {
    const origin = this.anchorOrigin(role);
    if (origin === null) {
      return null;
    }
    const forward = new THREE.Vector3(1, 0, 0);
    if (this.track !== null) {
      this.inertial.copy(origin);
      this.anchors.root.worldToLocal(this.inertial);
      const stationMeters = this.track.stationNearestToInertialPosition(this.inertial);
      const tangent = this.track.trackFrameAtStation(stationMeters).tangent;
      // Inertial (x, y, z) maps to Three.js (x, -z, y); keep the horizontal part.
      forward.set(tangent.x, 0, tangent.y);
    } else if (role === 'bogie' && this.anchors.bogie !== null) {
      forward.set(1, 0, 0).applyQuaternion(this.anchors.bogie.getWorldQuaternion(new THREE.Quaternion()));
      forward.y = 0;
    } else if (this.anchors.carbody !== null) {
      forward.set(1, 0, 0).applyQuaternion(this.anchors.carbody.getWorldQuaternion(new THREE.Quaternion()));
      forward.y = 0;
    }
    if (forward.lengthSq() < 1e-12) {
      forward.set(1, 0, 0);
    }
    forward.normalize();
    const left = new THREE.Vector3().crossVectors(up, forward).normalize();
    return { origin, forward, left };
  }

  private toWorld(anchorFrame: AnchorFrame, local: THREE.Vector3, out: THREE.Vector3): THREE.Vector3 {
    return out
      .copy(anchorFrame.origin)
      .addScaledVector(anchorFrame.forward, local.x)
      .addScaledVector(anchorFrame.left, local.y)
      .addScaledVector(up, local.z);
  }

  private toLocal(anchorFrame: AnchorFrame, world: THREE.Vector3, out: THREE.Vector3): THREE.Vector3 {
    this.scratch.copy(world).sub(anchorFrame.origin);
    return out.set(this.scratch.dot(anchorFrame.forward), this.scratch.dot(anchorFrame.left), this.scratch.y);
  }

  /** Camera and target of a preset, in the local frame, for the current viewport. */
  private destination(spec: PresetSpec, freeFraction: number): { camera: THREE.Vector3; target: THREE.Vector3 } {
    const target = new THREE.Vector3(...spec.target);
    const direction = new THREE.Vector3(...spec.direction).normalize();
    const halfWidth = Math.tan(THREE.MathUtils.degToRad(spec.fov / 2)) * this.camera.aspect * Math.max(0.35, freeFraction);
    const distance = spec.fitWidth / (2 * halfWidth);
    return { camera: target.clone().addScaledVector(direction, distance), target };
  }

  /** Moves to a preset; returns false and changes nothing when the preset's anchor is not bound. */
  applyPreset(preset: ViewPreset, freeFraction: number, immediate: boolean): boolean {
    if (!this.presetAvailable(preset)) {
      return false;
    }
    const spec = presets[preset];
    const anchorFrame = this.anchorFrameFor(spec.anchor);
    if (anchorFrame === null) {
      return false;
    }
    this.preset = preset;
    if (immediate) {
      const destination = this.destination(spec, freeFraction);
      this.toWorld(anchorFrame, destination.camera, this.camera.position);
      this.toWorld(anchorFrame, destination.target, this.controls.target);
      this.camera.fov = spec.fov;
      this.camera.updateProjectionMatrix();
      this.tween = null;
    } else {
      this.tween = {
        startCamera: this.toLocal(anchorFrame, this.camera.position, new THREE.Vector3()),
        startTarget: this.toLocal(anchorFrame, this.controls.target, new THREE.Vector3()),
        startFov: this.camera.fov,
        elapsed: 0,
        duration: 0.9,
      };
    }
    this.lastAnchorFrame = anchorFrame;
    this.controls.update();
    return true;
  }

  /** Called once per rendered frame after body poses are written. */
  update(wallDeltaSeconds: number, freeFraction: number): void {
    const spec = presets[this.preset];
    const anchorFrame = this.anchorFrameFor(spec.anchor);
    if (anchorFrame === null) {
      return;
    }
    if (this.tween !== null) {
      this.tween.elapsed += wallDeltaSeconds;
      const s = easeInOutCubic(Math.min(1, this.tween.elapsed / this.tween.duration));
      const destination = this.destination(spec, freeFraction);
      const cameraLocal = this.tween.startCamera.clone().lerp(destination.camera, s);
      const targetLocal = this.tween.startTarget.clone().lerp(destination.target, s);
      this.toWorld(anchorFrame, cameraLocal, this.camera.position);
      this.toWorld(anchorFrame, targetLocal, this.controls.target);
      this.camera.fov = THREE.MathUtils.lerp(this.tween.startFov, spec.fov, s);
      this.camera.updateProjectionMatrix();
      if (s >= 1) {
        this.tween = null;
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
