import * as THREE from 'three';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';

import type { WheelVisual } from '../record/visual_definition.ts';
import type { ScenePalette } from './materials.ts';

// A schematic railway wheel turned about its spin axis. The section is written
// in (radius, axial) coordinates with the axial coordinate measured outwards
// from the tape circle, which sits on the placement datum. The flange is on the
// back face, the side facing the wheel body origin: for a wheel pair that is
// the inside of the track. The wheel adds no motion of its own; the body pose
// from the record already carries its spin.

const segments = 72;

type SectionPoint = [radius: number, axial: number];

function lathe(points: SectionPoint[]): THREE.BufferGeometry {
  return new THREE.LatheGeometry(
    points.map(([radius, axial]) => new THREE.Vector2(radius, axial)),
    segments,
  );
}

function merged(strips: SectionPoint[][]): THREE.BufferGeometry {
  const geometry = mergeGeometries(strips.map(lathe));
  if (geometry === null) {
    throw new Error('wheel section strips could not be merged');
  }
  return geometry;
}

export interface WheelSection {
  body: THREE.BufferGeometry;
  tread: THREE.BufferGeometry;
  /** Axial coordinate of the hub's outer face, where a label anchors. */
  hubOuterAxial: number;
  /** Axial coordinate of the rim's outer face. */
  outerFaceAxial: number;
}

/** Builds the two surfaces of one wheel section: the painted body and the running band. */
export function buildWheelSection(rollingRadius: number, visual: WheelVisual): WheelSection {
  const r0 = rollingRadius;
  const back = -visual.backFaceOffsetMeters;
  const outer = visual.widthMeters - visual.backFaceOffsetMeters;
  const rimInner = r0 - visual.rimDepthMeters;
  const flangeTop = r0 + visual.flangeHeightMeters;
  const flangeEnd = back + visual.flangeThicknessMeters;
  const webCentre = 0.5 * (back + outer) - 0.012;
  const webInner = webCentre - 0.5 * visual.webThicknessMeters;
  const webOuter = webCentre + 0.5 * visual.webThicknessMeters;
  const hubInner = webCentre - 0.5 * visual.hubLengthMeters;
  const hubOuter = webCentre + 0.5 * visual.hubLengthMeters;
  const hub = visual.hubRadiusMeters;
  const conicity = 1 / 40;
  const treadRadius = (axial: number): number => r0 - axial * conicity;

  // The running band: back-face top, flange, root fillet, coned tread, chamfer.
  const flangeWidth = flangeEnd - back;
  const tread: SectionPoint[][] = [
    [
      [flangeTop - 0.008, back],
      [flangeTop - 0.002, back + 0.12 * flangeWidth],
      [flangeTop, back + 0.34 * flangeWidth],
      [flangeTop - 0.004, back + 0.58 * flangeWidth],
      [r0 + 0.4 * visual.flangeHeightMeters, back + 0.86 * flangeWidth],
      [r0 + 0.14 * visual.flangeHeightMeters, flangeEnd + 0.003],
      [treadRadius(flangeEnd + 0.012) + 0.0008, flangeEnd + 0.012],
      [treadRadius(0), 0],
      [treadRadius(outer - 0.006), outer - 0.006],
    ],
    [
      [treadRadius(outer - 0.006), outer - 0.006],
      [treadRadius(outer) - 0.005, outer],
    ],
  ];
  // The painted body, split at every sharp edge so each face keeps flat shading.
  const body: SectionPoint[][] = [
    [[0, hubInner], [hub, hubInner]],
    [[hub, hubInner], [hub, webInner]],
    [[hub, webInner], [rimInner, webInner]],
    [[rimInner, webInner], [rimInner, back]],
    [[rimInner, back], [flangeTop - 0.008, back]],
    [[treadRadius(outer) - 0.005, outer], [rimInner, outer]],
    [[rimInner, outer], [rimInner, webOuter]],
    [[rimInner, webOuter], [hub, webOuter]],
    [[hub, webOuter], [hub, hubOuter]],
    [[hub, hubOuter], [0, hubOuter]],
  ];
  return { body: merged(body), tread: merged(tread), hubOuterAxial: hubOuter, outerFaceAxial: outer };
}

export interface BuiltWheel {
  group: THREE.Group;
  treadMesh: THREE.Mesh;
  /** On the spin axis at the hub's outer face; it does not move as the wheel turns. */
  labelAnchor: THREE.Object3D;
}

/** The axis pointing away from the wheel body origin, i.e. towards the wheel's outer face. */
export function outwardAxis(datum: [number, number, number], spinAxis: [number, number, number]): THREE.Vector3 {
  const axis = new THREE.Vector3(...spinAxis).normalize();
  const along = new THREE.Vector3(...datum).dot(axis);
  return along < 0 ? axis.negate() : axis;
}

export function buildWheel(
  section: WheelSection,
  rollingRadius: number,
  datum: [number, number, number],
  spinAxis: [number, number, number],
  visual: WheelVisual,
  palette: ScenePalette,
): BuiltWheel {
  const group = new THREE.Group();
  const orient = new THREE.Group();
  orient.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), outwardAxis(datum, spinAxis));
  group.add(orient);
  group.position.set(...datum);

  const bodyMesh = new THREE.Mesh(section.body, palette.wheelBody);
  const treadMesh = new THREE.Mesh(section.tread, palette.wheelTread);
  for (const mesh of [bodyMesh, treadMesh]) {
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    orient.add(mesh);
  }
  if (visual.rimMarker) {
    // A painted mark on the outer rim face; it turns only because the body does.
    const marker = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.004, 0.055), palette.byAppearance.accent);
    marker.position.set(rollingRadius - visual.rimDepthMeters * 0.55, section.outerFaceAxial + 0.002, 0);
    orient.add(marker);
  }
  const labelAnchor = new THREE.Object3D();
  labelAnchor.position.set(0, section.hubOuterAxial, 0);
  orient.add(labelAnchor);
  return { group, treadMesh, labelAnchor };
}
