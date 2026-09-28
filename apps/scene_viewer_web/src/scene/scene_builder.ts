import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';

import type { SceneRecord } from '../record/scene_record.ts';
import { parseVisualDefinition, type PartLabel, type VisualDefinition, type VisualPart, type WheelVisual } from '../record/visual_definition.ts';
import { createPalette, stageColors, type ScenePalette } from './materials.ts';
import { buildTrack } from './track_builder.ts';
import { TrackModel } from './track_model.ts';
import { buildWheel, buildWheelSection, type WheelSection } from './wheel_geometry.ts';

// Builds the Three.js objects once from the record. Every body is an empty
// Object3D whose pose is written from the frames; parts hang under their body
// in that body's own frame. The root group turns the ORVD inertial frame
// (x forward, y right, z down) into Three.js Y-up: a rotation of +90 degrees
// about x maps (x, y, z)_I to (x, -z, y)_V, so nothing else permutes axes.

/** `body_axes` stands in for the vehicle when the record has no visual definition and is always shown. */
export type PartGroup = 'carbody' | 'running_gear' | 'wheels' | 'track' | 'axes' | 'body_axes';

export interface ScenePart {
  name: string;
  bodyName: string;
  group: PartGroup;
  object: THREE.Object3D;
}

export interface SceneLabel {
  en: string;
  zh: string;
  anchor: THREE.Object3D;
  /** Characteristic size of the labelled part, for hiding labels of parts too small to see. */
  sizeMeters: number;
}

export type CarbodyMode = 'xray' | 'solid' | 'hidden';

/** What the display switches ask for; parts named in the hidden set stay hidden regardless. */
export interface DisplayState {
  carbody: CarbodyMode;
  track: boolean;
  axes: boolean;
}

export interface BuiltScene {
  root: THREE.Group;
  bodyObjects: Map<string, THREE.Object3D>;
  parts: ScenePart[];
  visualDefinition: VisualDefinition | null;
  labels: SceneLabel[];
  track: TrackModel | null;
  palette: ScenePalette;
  setDisplay: (state: DisplayState) => void;
  /** Parts hidden by name (the `hide` deep link); they stay hidden whatever the display switches say. */
  setUserHidden: (names: ReadonlySet<string>) => void;
  /**
   * Colours each wheel's running band by its loaded contact patch count: 1 plain,
   * 0 loss of contact, 2 or more two-point contact, NaN not ready. null restores the plain band.
   */
  setContactStates: (patchCounts: ArrayLike<number> | null) => void;
  dispose: () => void;
}

function cylinderBetween(
  start: [number, number, number],
  end: [number, number, number],
  radius: number,
  material: THREE.Material,
): THREE.Mesh {
  const from = new THREE.Vector3(...start);
  const to = new THREE.Vector3(...end);
  const direction = to.clone().sub(from);
  const length = direction.length();
  if (!(length > 0)) {
    throw new Error('a cylinder part needs two distinct end points');
  }
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, length, 28), material);
  mesh.position.copy(from).add(to).multiplyScalar(0.5);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction.normalize());
  return mesh;
}

function partCentre(part: VisualPart): [number, number, number] {
  switch (part.kind) {
    case 'box':
    case 'sphere':
      return part.centerInBodyFrameMeters;
    case 'cylinder':
      return [
        0.5 * (part.startInBodyFrameMeters[0] + part.endInBodyFrameMeters[0]),
        0.5 * (part.startInBodyFrameMeters[1] + part.endInBodyFrameMeters[1]),
        0.5 * (part.startInBodyFrameMeters[2] + part.endInBodyFrameMeters[2]),
      ];
    case 'axes':
      return part.originInBodyFrameMeters;
  }
}

function buildPart(part: VisualPart, palette: ScenePalette): THREE.Object3D {
  if (part.kind === 'axes') {
    const axes = new THREE.AxesHelper(part.lengthMeters);
    axes.setColors(new THREE.Color(stageColors.accent), new THREE.Color(stageColors.ok), new THREE.Color(stageColors.ink));
    axes.position.set(...part.originInBodyFrameMeters);
    return axes;
  }
  const material = palette.byAppearance[part.appearance];
  let mesh: THREE.Mesh;
  switch (part.kind) {
    case 'box': {
      const [x, y, z] = part.sizeMeters;
      const radius = Math.min(part.cornerRadiusMeters, 0.5 * Math.min(x, y, z) - 1e-4);
      const geometry =
        radius > 0 ? new RoundedBoxGeometry(x, y, z, part.appearance === 'shell' ? 8 : 3, radius) : new THREE.BoxGeometry(x, y, z);
      mesh = new THREE.Mesh(geometry, material);
      mesh.position.set(...part.centerInBodyFrameMeters);
      break;
    }
    case 'cylinder':
      mesh = cylinderBetween(part.startInBodyFrameMeters, part.endInBodyFrameMeters, part.radiusMeters, material);
      break;
    case 'sphere':
      mesh = new THREE.Mesh(new THREE.SphereGeometry(part.radiusMeters, 20, 14), material);
      mesh.position.set(...part.centerInBodyFrameMeters);
      break;
  }
  const opaque = part.appearance === 'structure' || part.appearance === 'accent' || part.appearance === 'dark';
  mesh.castShadow = opaque;
  mesh.receiveShadow = opaque;
  // Draw the see-through carbody layers after everything else, floor first.
  mesh.renderOrder = { shell: 3, glass: 2, floor: 1, structure: 0, accent: 0, dark: 0 }[part.appearance];
  return mesh;
}

function partSize(part: VisualPart): number {
  switch (part.kind) {
    case 'box':
      return Math.max(...part.sizeMeters);
    case 'cylinder': {
      const [x0, y0, z0] = part.startInBodyFrameMeters;
      const [x1, y1, z1] = part.endInBodyFrameMeters;
      return Math.max(Math.hypot(x1 - x0, y1 - y0, z1 - z0), 2 * part.radiusMeters);
    }
    case 'sphere':
      return 2 * part.radiusMeters;
    case 'axes':
      return part.lengthMeters;
  }
}

/** Thin outline of a rounded box: both end sections and the four long edges, as in a line drawing. */
function roundedBoxOutline(size: [number, number, number], radius: number): THREE.BufferGeometry {
  const [length, width, height] = size;
  const points: number[] = [];
  const section = (x: number): void => {
    const loop: [number, number][] = [];
    const corners: [number, number, number][] = [
      [width / 2 - radius, height / 2 - radius, 0],
      [-(width / 2 - radius), height / 2 - radius, Math.PI / 2],
      [-(width / 2 - radius), -(height / 2 - radius), Math.PI],
      [width / 2 - radius, -(height / 2 - radius), (3 * Math.PI) / 2],
    ];
    for (const [cy, cz, start] of corners) {
      for (let step = 0; step <= 6; ++step) {
        const angle = start + (step / 6) * (Math.PI / 2);
        loop.push([cy + radius * Math.cos(angle), cz + radius * Math.sin(angle)]);
      }
    }
    loop.forEach(([y, z], index) => {
      const [ny, nz] = loop[(index + 1) % loop.length] as [number, number];
      points.push(x, y, z, x, ny, nz);
    });
  };
  const inset = length / 2 - radius;
  section(inset);
  section(-inset);
  const diagonal = radius * (1 - Math.SQRT1_2);
  for (const [y, z] of [
    [width / 2 - diagonal, height / 2 - diagonal],
    [-(width / 2 - diagonal), height / 2 - diagonal],
    [-(width / 2 - diagonal), -(height / 2 - diagonal)],
    [width / 2 - diagonal, -(height / 2 - diagonal)],
  ] as [number, number][]) {
    points.push(-inset, y, z, inset, y, z);
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(points, 3));
  return geometry;
}

function labelAnchor(label: PartLabel, fallback: [number, number, number]): THREE.Object3D {
  const anchor = new THREE.Object3D();
  anchor.position.set(...(label.anchorInBodyFrameMeters ?? fallback));
  return anchor;
}

/** A schematic wheel section in proportion to the rolling radius, for records without a visual definition. */
function defaultWheelVisual(radius: number): WheelVisual {
  return {
    widthMeters: 0.314 * radius,
    backFaceOffsetMeters: 0.163 * radius,
    flangeHeightMeters: 0.065 * radius,
    flangeThicknessMeters: 0.074 * radius,
    rimDepthMeters: 0.105 * radius,
    webThicknessMeters: 0.07 * radius,
    hubRadiusMeters: 0.256 * radius,
    hubLengthMeters: 0.3 * radius,
    rimMarker: true,
    labels: [],
  };
}

type CarbodyRole = 'shell' | 'glass' | 'floor' | null;

export function buildScene(record: SceneRecord): BuiltScene {
  const palette = createPalette();
  const root = new THREE.Group();
  root.rotation.x = Math.PI / 2;
  const bodyObjects = new Map<string, THREE.Object3D>();
  const parts: ScenePart[] = [];
  const labels: SceneLabel[] = [];
  for (const body of record.bodies) {
    const object = new THREE.Object3D();
    object.name = body.name;
    root.add(object);
    bodyObjects.set(body.name, object);
  }

  const shells: THREE.Mesh[] = [];
  const roles = new Map<ScenePart, { role: CarbodyRole; outline: THREE.LineSegments | null }>();
  const outlines: THREE.LineSegments[] = [];
  const outlineMaterial = new THREE.LineBasicMaterial({ color: '#9A9185', transparent: true, opacity: 0.42, depthWrite: false });
  let visualDefinition: VisualDefinition | null = null;
  if (record.visualDefinitionText !== null) {
    visualDefinition = parseVisualDefinition(record.visualDefinitionText, new Set(bodyObjects.keys()));
    for (const part of visualDefinition.parts) {
      const object = buildPart(part, palette);
      const body = bodyObjects.get(part.bodyName);
      body?.add(object);
      let group: PartGroup = part.bodyName === 'carbody' ? 'carbody' : 'running_gear';
      let role: CarbodyRole = null;
      let outline: THREE.LineSegments | null = null;
      if (part.kind === 'axes') {
        group = 'axes';
      } else if (part.appearance === 'shell' && object instanceof THREE.Mesh) {
        role = 'shell';
        shells.push(object);
        if (part.kind === 'box') {
          outline = new THREE.LineSegments(roundedBoxOutline(part.sizeMeters, Math.min(part.cornerRadiusMeters, 0.5 * Math.min(...part.sizeMeters) - 1e-4)), outlineMaterial);
          outline.position.set(...part.centerInBodyFrameMeters);
          outline.renderOrder = 4;
          body?.add(outline);
          outlines.push(outline);
        }
      } else if (part.appearance === 'glass' || part.appearance === 'floor') {
        role = part.appearance;
      }
      const entry: ScenePart = { name: part.name, bodyName: part.bodyName, group, object };
      parts.push(entry);
      roles.set(entry, { role, outline });
      if (part.kind !== 'axes' && part.label !== null && body !== undefined) {
        const anchor = labelAnchor(part.label, partCentre(part));
        body.add(anchor);
        labels.push({ en: part.label.en, zh: part.label.zh, anchor, sizeMeters: partSize(part) });
      }
    }
  } else {
    // Without a visual definition every body is shown as its own axes, and the
    // wheels as plain schematic wheels from their recorded placements.
    for (const [name, object] of bodyObjects) {
      const axes = new THREE.AxesHelper(0.5);
      axes.setColors(new THREE.Color(stageColors.accent), new THREE.Color(stageColors.ok), new THREE.Color(stageColors.ink));
      object.add(axes);
      parts.push({ name: `${name} axes`, bodyName: name, group: 'body_axes', object: axes });
    }
  }

  const treads: THREE.Mesh[] = [];
  const sections = new Map<number, WheelSection>();
  record.wheelPlacements.forEach((placement) => {
    const body = bodyObjects.get(placement.wheelBodyName);
    if (body === undefined) {
      throw new Error(`wheel placement names unknown body '${placement.wheelBodyName}'`);
    }
    const radius = placement.nominalRollingRadiusMeters;
    const wheelVisual = visualDefinition?.wheelVisual ?? defaultWheelVisual(radius);
    let section = sections.get(radius);
    if (section === undefined) {
      section = buildWheelSection(radius, wheelVisual);
      sections.set(radius, section);
    }
    const wheel = buildWheel(section, radius, placement.datumInWheelBodyFrameMeters, placement.spinAxisInWheelBodyFrame, wheelVisual, palette);
    body.add(wheel.group);
    treads.push(wheel.treadMesh);
    parts.push({ name: placement.interfaceName, bodyName: placement.wheelBodyName, group: 'wheels', object: wheel.group });
    const wheelLabel = wheelVisual.labels.find((entry) => entry.interfaceName === placement.interfaceName);
    if (wheelLabel !== undefined) {
      labels.push({ en: wheelLabel.en, zh: wheelLabel.zh, anchor: wheel.labelAnchor, sizeMeters: 2 * radius });
    }
  });

  let track: TrackModel | null = null;
  if (record.track !== null) {
    track = new TrackModel(record.track);
    const built = buildTrack(track, palette);
    root.add(built.group);
    root.add(built.ground);
    parts.push({ name: 'track', bodyName: '', group: 'track', object: built.group });
    parts.push({ name: 'ground', bodyName: '', group: 'track', object: built.ground });
  }

  let display: DisplayState = { carbody: 'xray', track: true, axes: false };
  let userHidden: ReadonlySet<string> = new Set();
  const refreshVisibility = (): void => {
    for (const part of parts) {
      const extra = roles.get(part);
      let visible = !userHidden.has(part.name);
      if (part.group === 'axes') {
        visible &&= display.axes;
      } else if (part.group === 'track') {
        visible &&= display.track;
      }
      if (extra?.role === 'shell' || extra?.role === 'glass') {
        visible &&= display.carbody !== 'hidden';
      } else if (extra?.role === 'floor') {
        visible &&= display.carbody === 'xray';
      }
      part.object.visible = visible;
      if (extra?.outline !== null && extra?.outline !== undefined) {
        extra.outline.visible = visible;
      }
    }
    const solid = display.carbody === 'solid';
    for (const shell of shells) {
      shell.material = solid ? palette.solidShell : palette.byAppearance.shell;
      shell.castShadow = solid;
      shell.receiveShadow = solid;
      shell.renderOrder = solid ? 0 : 3;
    }
    outlineMaterial.opacity = solid ? 0.3 : 0.42;
  };
  const setDisplay = (state: DisplayState): void => {
    display = { ...state };
    refreshVisibility();
  };
  const setUserHidden = (names: ReadonlySet<string>): void => {
    userHidden = new Set(names);
    refreshVisibility();
  };
  refreshVisibility();

  const setContactStates = (patchCounts: ArrayLike<number> | null): void => {
    treads.forEach((tread, index) => {
      const count = patchCounts === null ? 1 : patchCounts[index];
      tread.material =
        count === undefined || Number.isNaN(count)
          ? palette.wheelTreadUnknown
          : count === 1
            ? palette.wheelTread
            : count === 0
              ? palette.wheelTreadLoss
              : palette.wheelTreadTwoPoint;
    });
  };

  const dispose = (): void => {
    const geometries = new Set<THREE.BufferGeometry>();
    root.traverse((object) => {
      if (object instanceof THREE.AxesHelper) {
        object.dispose();
        return;
      }
      if (object instanceof THREE.Mesh || object instanceof THREE.Line || object instanceof THREE.Points) {
        geometries.add(object.geometry as THREE.BufferGeometry);
      }
      if (object instanceof THREE.Sprite) {
        object.material.map?.dispose();
        object.material.dispose();
      }
    });
    geometries.forEach((geometry) => geometry.dispose());
    outlineMaterial.dispose();
    palette.dispose();
  };

  return { root, bodyObjects, parts, visualDefinition, labels, track, palette, setDisplay, setUserHidden, setContactStates, dispose };
}
