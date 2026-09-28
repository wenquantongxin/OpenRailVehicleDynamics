import * as THREE from 'three';
import type { OrbitControls } from 'three/addons/controls/OrbitControls.js';

import type { TrackModel } from './track_model.ts';

// Camera presets stated in the line's local frame at the vehicle (forward,
// left, up), so "front" stays in front of the vehicle through a curve. The
// heading comes from the recorded track frame at the anchor's station, not
// from the body's own yaw, so the view does not sway with the carbody.

export type ViewPreset = 'overview' | 'bogie' | 'headon' | 'plan' | 'side';

export const viewPresets: ViewPreset[] = ['overview', 'bogie', 'headon', 'plan', 'side'];

interface PresetSpec {
  anchor: 'carbody' | 'bogie';
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

const up = new THREE.Vector3(0, 1, 0);

export interface Anchors {
  root: THREE.Object3D;
  carbody: THREE.Object3D | null;
  bogie: THREE.Object3D | null;
}

interface LocalFrame {
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
  private lastFrame: LocalFrame | null = null;
  private readonly scratch = new THREE.Vector3();
  private readonly inertial = new THREE.Vector3();

  private readonly camera: THREE.PerspectiveCamera;
  private readonly controls: OrbitControls;
  private readonly anchors: Anchors;
  private readonly track: TrackModel | null;

  constructor(camera: THREE.PerspectiveCamera, controls: OrbitControls, anchors: Anchors, track: TrackModel | null) {
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

  /** The anchor's world position and the horizontal track heading there. */
  private frameFor(anchorName: PresetSpec['anchor']): LocalFrame | null {
    const anchor = anchorName === 'bogie' ? (this.anchors.bogie ?? this.anchors.carbody) : this.anchors.carbody;
    if (anchor === null) {
      return null;
    }
    const origin = anchor.getWorldPosition(new THREE.Vector3());
    const forward = new THREE.Vector3(1, 0, 0);
    if (this.track !== null) {
      this.inertial.copy(origin);
      this.anchors.root.worldToLocal(this.inertial);
      const station = this.track.stationNearest(this.inertial);
      const tangent = this.track.frameAt(station).tangent;
      // Inertial (x, y, z) maps to Three.js (x, -z, y); keep the horizontal part.
      forward.set(tangent.x, 0, tangent.y);
    } else {
      forward.set(1, 0, 0).applyQuaternion(anchor.getWorldQuaternion(new THREE.Quaternion()));
      forward.y = 0;
    }
    if (forward.lengthSq() < 1e-12) {
      forward.set(1, 0, 0);
    }
    forward.normalize();
    const left = new THREE.Vector3().crossVectors(up, forward).normalize();
    return { origin, forward, left };
  }

  private toWorld(frame: LocalFrame, local: THREE.Vector3, out: THREE.Vector3): THREE.Vector3 {
    return out
      .copy(frame.origin)
      .addScaledVector(frame.forward, local.x)
      .addScaledVector(frame.left, local.y)
      .addScaledVector(up, local.z);
  }

  private toLocal(frame: LocalFrame, world: THREE.Vector3, out: THREE.Vector3): THREE.Vector3 {
    this.scratch.copy(world).sub(frame.origin);
    return out.set(this.scratch.dot(frame.forward), this.scratch.dot(frame.left), this.scratch.y);
  }

  /** Camera and target of a preset, in the local frame, for the current viewport. */
  private destination(spec: PresetSpec, freeFraction: number): { camera: THREE.Vector3; target: THREE.Vector3 } {
    const target = new THREE.Vector3(...spec.target);
    const direction = new THREE.Vector3(...spec.direction).normalize();
    const halfWidth = Math.tan(THREE.MathUtils.degToRad(spec.fov / 2)) * this.camera.aspect * Math.max(0.35, freeFraction);
    const distance = spec.fitWidth / (2 * halfWidth);
    return { camera: target.clone().addScaledVector(direction, distance), target };
  }

  applyPreset(preset: ViewPreset, freeFraction: number, immediate: boolean): void {
    this.preset = preset;
    const spec = presets[preset];
    const frame = this.frameFor(spec.anchor);
    if (frame === null) {
      return;
    }
    if (immediate) {
      const destination = this.destination(spec, freeFraction);
      this.toWorld(frame, destination.camera, this.camera.position);
      this.toWorld(frame, destination.target, this.controls.target);
      this.camera.fov = spec.fov;
      this.camera.updateProjectionMatrix();
      this.tween = null;
    } else {
      this.tween = {
        startCamera: this.toLocal(frame, this.camera.position, new THREE.Vector3()),
        startTarget: this.toLocal(frame, this.controls.target, new THREE.Vector3()),
        startFov: this.camera.fov,
        elapsed: 0,
        duration: 0.9,
      };
    }
    this.lastFrame = frame;
    this.controls.update();
  }

  /** Called once per rendered frame after body poses are written. */
  update(wallDeltaSeconds: number, freeFraction: number): void {
    const spec = presets[this.preset];
    const frame = this.frameFor(spec.anchor);
    if (frame === null) {
      return;
    }
    if (this.tween !== null) {
      this.tween.elapsed += wallDeltaSeconds;
      const s = easeInOutCubic(Math.min(1, this.tween.elapsed / this.tween.duration));
      const destination = this.destination(spec, freeFraction);
      const cameraLocal = this.tween.startCamera.clone().lerp(destination.camera, s);
      const targetLocal = this.tween.startTarget.clone().lerp(destination.target, s);
      this.toWorld(frame, cameraLocal, this.camera.position);
      this.toWorld(frame, targetLocal, this.controls.target);
      this.camera.fov = THREE.MathUtils.lerp(this.tween.startFov, spec.fov, s);
      this.camera.updateProjectionMatrix();
      if (s >= 1) {
        this.tween = null;
      }
    } else if (this.follow && this.lastFrame !== null) {
      // Carry the camera with the anchor: same offset, turned with the heading.
      const cameraLocal = this.toLocal(this.lastFrame, this.camera.position, new THREE.Vector3());
      const targetLocal = this.toLocal(this.lastFrame, this.controls.target, new THREE.Vector3());
      this.toWorld(frame, cameraLocal, this.camera.position);
      this.toWorld(frame, targetLocal, this.controls.target);
    }
    this.lastFrame = frame;
    this.controls.update();
  }
}
