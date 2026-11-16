import * as THREE from 'three';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';

import type { WheelSectionSizes } from '../visual/visual_definition.ts';

// A schematic railway wheel turned about its spin axis, built as the scene
// viewer builds it (apps/scene_viewer_web/src/scene/wheel_geometry.ts), without
// the rim marker and the label anchor: nothing turns in this view. The section
// is written in (radius, axial) coordinates with the axial coordinate measured
// outwards from the tape circle, which sits on the wheel's datum. The flange is
// on the back face, the side facing the carrier's centre: for a pair of wheels,
// the inside of the track. The lathe's axis is +y.

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

/** The two surfaces of one wheel: the painted body and the running band. */
export interface WheelSection {
  body: THREE.BufferGeometry;
  tread: THREE.BufferGeometry;
}

/** The section of a usual schematic wheel, for wheels drawn without a visual definition. */
export const standardWheelSectionSizes: WheelSectionSizes = {
  widthMeters: 0.135,
  backFaceOffsetMeters: 0.07,
  flangeHeightMeters: 0.028,
  flangeThicknessMeters: 0.032,
  rimDepthMeters: 0.045,
  webThicknessMeters: 0.03,
  hubRadiusMeters: 0.11,
  hubLengthMeters: 0.13,
};

export function buildWheelSection(rollingRadius: number, sizes: WheelSectionSizes): WheelSection {
  const r0 = rollingRadius;
  const back = -sizes.backFaceOffsetMeters;
  const outer = sizes.widthMeters - sizes.backFaceOffsetMeters;
  const rimInner = r0 - sizes.rimDepthMeters;
  const flangeTop = r0 + sizes.flangeHeightMeters;
  const flangeEnd = back + sizes.flangeThicknessMeters;
  const webCentre = 0.5 * (back + outer) - 0.012;
  const webInner = webCentre - 0.5 * sizes.webThicknessMeters;
  const webOuter = webCentre + 0.5 * sizes.webThicknessMeters;
  const hubInner = webCentre - 0.5 * sizes.hubLengthMeters;
  const hubOuter = webCentre + 0.5 * sizes.hubLengthMeters;
  const hub = sizes.hubRadiusMeters;
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
      [r0 + 0.4 * sizes.flangeHeightMeters, back + 0.86 * flangeWidth],
      [r0 + 0.14 * sizes.flangeHeightMeters, flangeEnd + 0.003],
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
  return { body: merged(body), tread: merged(tread) };
}
