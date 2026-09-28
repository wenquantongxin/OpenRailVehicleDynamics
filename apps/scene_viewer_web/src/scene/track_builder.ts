import * as THREE from 'three';

import { chainage } from '../ui/format.ts';
import type { ScenePalette } from './materials.ts';
import { stageColors } from './materials.ts';
import type { TrackFrame, TrackModel } from './track_model.ts';

// Display geometry for the sampled line. Every section is written in Track-T
// coordinates (lateral offset to the right, depth offset downwards) about a
// recorded rail datum or the centreline, then placed with the station's
// recorded origin and rotation, so cant is carried by the rotation itself.
// Rails, pads and clips sit on the recorded rail datums; sleepers and ballast
// are centred between them and scale with the recorded datum spacing. Rail,
// sleeper and ballast shapes are schematic (a 60E1-like rail, B70-like
// sleepers at 0.6 m); the record carries only the design line, without
// irregularity.
//
// The ground is a plane under the lowest point of the line. A formation strip
// follows the line at groundDepth below the centreline and slopes down to the
// plane wherever the line runs higher, so a graded or raised line stands on an
// embankment instead of floating or being buried.

const railDepth = 0.172;
const padThickness = 0.012;
const sleeperSpacing = 0.6;
const sleeperHeight = 0.21;
const sleeperBottomWidth = 0.28;
const sleeperTopWidth = 0.22;
const ballastTopDepth = 0.215;
/** Depth of the formation below the centreline. */
export const groundDepth = 0.62;
/** Side slope of the formation, horizontal run per unit height. */
const formationSlope = 1.5;
const groundSize = 4000;

/**
 * Stations at which the ballast and formation strips are sampled: from the
 * first station every `stepMeters`, then the exact last station, appended when
 * the last regular sample stops short of it and substituted for that sample
 * when it lies within tolerance. The result is increasing, has no duplicate
 * and always reaches both ends, so no tail shorter than one step is dropped
 * and no degenerate tail face is drawn.
 */
export function stripStationsMeters(firstStationMeters: number, lastStationMeters: number, stepMeters: number): number[] {
  if (!(stepMeters > 0)) {
    throw new Error('the strip sampling step must be positive');
  }
  if (!(lastStationMeters >= firstStationMeters)) {
    throw new Error('the last strip station must not precede the first');
  }
  const tolerance = 1e-9;
  const stations: number[] = [firstStationMeters];
  if (lastStationMeters === firstStationMeters) {
    return stations;
  }
  for (let k = 1; ; ++k) {
    const stationMeters = firstStationMeters + k * stepMeters;
    if (stationMeters > lastStationMeters - tolerance) {
      break;
    }
    stations.push(stationMeters);
  }
  stations.push(lastStationMeters);
  return stations;
}

/** Half of a 60E1-like section, from the crown outwards and down to the foot. */
const railHalfSection: [number, number][] = [
  [0.0, 0.0],
  [0.012, 0.0003],
  [0.024, 0.0014],
  [0.031, 0.004],
  [0.0355, 0.01],
  [0.036, 0.016],
  [0.036, 0.036],
  [0.031, 0.046],
  [0.012, 0.054],
  [0.0083, 0.064],
  [0.0083, 0.13],
  [0.014, 0.142],
  [0.05, 0.1545],
  [0.075, 0.159],
  [0.075, railDepth],
];

function railSection(): [number, number][] {
  // Closed loop: right half from the crown down, then the left half back up.
  const right = railHalfSection;
  const left = [...railHalfSection].reverse().map(([v, w]) => [-v, w] as [number, number]);
  const loop = [...right, ...left.slice(0, -1)];
  return loop;
}

function signedArea(loop: [number, number][]): number {
  let area = 0;
  loop.forEach(([v0, w0], index) => {
    const [v1, w1] = loop[(index + 1) % loop.length] as [number, number];
    area += v0 * w1 - v1 * w0;
  });
  return 0.5 * area;
}

/** Extrudes a closed section along the table stations, with one flat normal per section edge. */
function extrudeSection(
  model: TrackModel,
  loop: [number, number][],
  offsetAt: (stationIndex: number) => [number, number],
  isHead: (w0: number, w1: number) => boolean,
): THREE.BufferGeometry {
  const stationsMeters = Array.from(model.stationsMeters);
  const edgeCount = loop.length;
  const orientation = signedArea(loop) > 0 ? 1 : -1;
  const positions: number[] = [];
  const normals: number[] = [];
  const headIndices: number[] = [];
  const bodyIndices: number[] = [];
  const trackFrame = model.trackFrameAtStation(stationsMeters[0] as number);
  const point = new THREE.Vector3();
  const normal = new THREE.Vector3();
  const vertexIndex = (stationIndex: number, edge: number, end: number): number => (stationIndex * edgeCount + edge) * 2 + end;
  stationsMeters.forEach((stationMeters, stationIndex) => {
    model.trackFrameAtStation(stationMeters, trackFrame);
    const [lateralOffset, depthOffset] = offsetAt(stationIndex);
    for (let edge = 0; edge < edgeCount; ++edge) {
      const [v0, w0] = loop[edge] as [number, number];
      const [v1, w1] = loop[(edge + 1) % edgeCount] as [number, number];
      const dv = v1 - v0;
      const dw = w1 - w0;
      const length = Math.hypot(dv, dw);
      const nv = (orientation * dw) / length;
      const nw = (-orientation * dv) / length;
      normal.copy(trackFrame.right).multiplyScalar(nv).addScaledVector(trackFrame.down, nw).normalize();
      for (const [v, w] of [[v0, w0], [v1, w1]] as const) {
        point
          .copy(trackFrame.position)
          .addScaledVector(trackFrame.right, lateralOffset + v)
          .addScaledVector(trackFrame.down, depthOffset + w);
        positions.push(point.x, point.y, point.z);
        normals.push(normal.x, normal.y, normal.z);
      }
    }
  });
  for (let stationIndex = 0; stationIndex + 1 < stationsMeters.length; ++stationIndex) {
    for (let edge = 0; edge < edgeCount; ++edge) {
      const a = vertexIndex(stationIndex, edge, 0);
      const b = vertexIndex(stationIndex, edge, 1);
      const c = vertexIndex(stationIndex + 1, edge, 0);
      const d = vertexIndex(stationIndex + 1, edge, 1);
      const [, w0] = loop[edge] as [number, number];
      const [, w1] = loop[(edge + 1) % edgeCount] as [number, number];
      const target = isHead(w0, w1) ? headIndices : bodyIndices;
      target.push(a, c, b, b, c, d);
    }
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometry.setAttribute('normal', new THREE.Float32BufferAttribute(normals, 3));
  geometry.setIndex([...headIndices, ...bodyIndices]);
  geometry.addGroup(0, headIndices.length, 0);
  geometry.addGroup(headIndices.length, bodyIndices.length, 1);
  orientFaces(geometry);
  return geometry;
}

/** Flips the winding when the first triangle's face normal disagrees with its vertex normals. */
function orientFaces(geometry: THREE.BufferGeometry): void {
  const index = geometry.getIndex();
  const position = geometry.getAttribute('position');
  const normal = geometry.getAttribute('normal');
  if (index === null || index.count < 3) {
    return;
  }
  // The first triangle with a real area decides; degenerate strips are skipped.
  const a = new THREE.Vector3();
  const b = new THREE.Vector3();
  const c = new THREE.Vector3();
  const face = new THREE.Vector3();
  let first = -1;
  for (let i = 0; i + 2 < index.count; i += 3) {
    a.fromBufferAttribute(position, index.getX(i));
    b.fromBufferAttribute(position, index.getX(i + 1));
    c.fromBufferAttribute(position, index.getX(i + 2));
    face.copy(b.sub(a)).cross(c.sub(a));
    if (face.lengthSq() > 1e-12) {
      first = i;
      break;
    }
  }
  if (first < 0) {
    return;
  }
  const vertexNormal = new THREE.Vector3().fromBufferAttribute(normal, index.getX(first));
  if (face.dot(vertexNormal) < 0) {
    const array = index.array as Uint32Array | Uint16Array;
    for (let i = 0; i < array.length; i += 3) {
      const swap = array[i + 1] as number;
      array[i + 1] = array[i + 2] as number;
      array[i + 2] = swap;
    }
    index.needsUpdate = true;
  }
}

/** Midpoint between the two recorded rail datums at a station, in Track-T: [lateral, depth]. */
function datumMidpoint(model: TrackModel, stationMeters: number): [number, number] {
  const [vl, wl] = model.railDatumOffsetAtStation(stationMeters, 'left');
  const [vr, wr] = model.railDatumOffsetAtStation(stationMeters, 'right');
  return [0.5 * (vl + vr), 0.5 * (wl + wr)];
}

function ballastGeometry(model: TrackModel): THREE.BufferGeometry {
  // Left slope, crib top and right slope, each a strip with its own normals.
  // Widths follow the recorded datum spacing; depths are below the rail datum level.
  const halfSpacing = 0.5 * model.medianRailDatumSpacingMeters;
  const profile: [number, number][] = [
    [-(halfSpacing + 2.2), 1.05],
    [-(halfSpacing + 0.95), ballastTopDepth],
    [halfSpacing + 0.95, ballastTopDepth],
    [halfSpacing + 2.2, 1.05],
  ];
  const positions: number[] = [];
  const normals: number[] = [];
  const uvs: number[] = [];
  const indices: number[] = [];
  const trackFrame = model.trackFrameAtStation(model.firstStationMeters);
  const point = new THREE.Vector3();
  const normal = new THREE.Vector3();
  const stripStations = stripStationsMeters(model.firstStationMeters, model.lastStationMeters, 1.0);
  const stripCount = stripStations.length;
  const faces = profile.length - 1;
  for (let stripIndex = 0; stripIndex < stripCount; ++stripIndex) {
    const stationMeters = stripStations[stripIndex] as number;
    model.trackFrameAtStation(stationMeters, trackFrame);
    const [vMid, wMid] = datumMidpoint(model, stationMeters);
    for (let face = 0; face < faces; ++face) {
      const [v0, w0] = profile[face] as [number, number];
      const [v1, w1] = profile[face + 1] as [number, number];
      const dv = v1 - v0;
      const dw = w1 - w0;
      const length = Math.hypot(dv, dw);
      // Outward (upward) normal of a strip running left to right.
      normal.copy(trackFrame.right).multiplyScalar(dw / length).addScaledVector(trackFrame.down, -dv / length).normalize();
      for (const [v, w] of [[v0, w0], [v1, w1]] as const) {
        point.copy(trackFrame.position).addScaledVector(trackFrame.right, vMid + v).addScaledVector(trackFrame.down, wMid + w);
        positions.push(point.x, point.y, point.z);
        normals.push(normal.x, normal.y, normal.z);
        uvs.push(v / 1.2, stationMeters / 1.2);
      }
    }
  }
  for (let stripIndex = 0; stripIndex + 1 < stripCount; ++stripIndex) {
    for (let face = 0; face < faces; ++face) {
      const a = (stripIndex * faces + face) * 2;
      const b = a + 1;
      const c = ((stripIndex + 1) * faces + face) * 2;
      const d = c + 1;
      indices.push(a, c, b, b, c, d);
    }
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometry.setAttribute('normal', new THREE.Float32BufferAttribute(normals, 3));
  geometry.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
  geometry.setIndex(indices);
  orientFaces(geometry);
  return geometry;
}

/** Horizontal unit vector to the right of the line at a station (the formation is not canted). */
function horizontalRight(trackFrame: Pick<TrackFrame, 'tangent' | 'right'>, out: THREE.Vector3): THREE.Vector3 {
  out.set(trackFrame.right.x, trackFrame.right.y, 0);
  if (out.lengthSq() < 1e-12) {
    out.set(-trackFrame.tangent.y, trackFrame.tangent.x, 0);
  }
  return out.normalize();
}

/** Ground texture coordinates in world metres, matching the ground plane's mapping. */
function groundUv(x: number, y: number, centreX: number, centreY: number): [number, number] {
  return [(x - centreX) / 10 + groundSize / 20, -(y - centreY) / 10 + groundSize / 20];
}

function formationGeometry(model: TrackModel, planeZ: number, halfWidth: number, centreX: number, centreY: number): THREE.BufferGeometry {
  const positions: number[] = [];
  const normals: number[] = [];
  const uvs: number[] = [];
  const indices: number[] = [];
  const trackFrame = model.trackFrameAtStation(model.firstStationMeters);
  const lateral = new THREE.Vector3();
  const normal = new THREE.Vector3();
  const stripStations = stripStationsMeters(model.firstStationMeters, model.lastStationMeters, 1.0);
  const stripCount = stripStations.length;
  const faces = 3;
  for (let stripIndex = 0; stripIndex < stripCount; ++stripIndex) {
    const stationMeters = stripStations[stripIndex] as number;
    model.trackFrameAtStation(stationMeters, trackFrame);
    horizontalRight(trackFrame, lateral);
    const topZ = trackFrame.position.z + groundDepth;
    const spread = formationSlope * Math.max(0, planeZ - topZ);
    // Cross-section (lateral u, inertial z): slope toe, top edge, top edge, slope toe.
    const section: [number, number][] = [
      [-halfWidth - spread, planeZ],
      [-halfWidth, topZ],
      [halfWidth, topZ],
      [halfWidth + spread, planeZ],
    ];
    for (let face = 0; face < faces; ++face) {
      const [u0, z0] = section[face] as [number, number];
      const [u1, z1] = section[face + 1] as [number, number];
      const du = u1 - u0;
      const dz = z1 - z0;
      const length = Math.hypot(du, dz);
      if (length > 1e-9) {
        // Upward normal of a strip running left to right (z points down).
        normal.copy(lateral).multiplyScalar(dz / length);
        normal.z += -du / length;
        normal.normalize();
      } else {
        normal.set(0, 0, -1);
      }
      for (const [u, z] of [[u0, z0], [u1, z1]] as const) {
        const x = trackFrame.position.x + lateral.x * u;
        const y = trackFrame.position.y + lateral.y * u;
        positions.push(x, y, z);
        normals.push(normal.x, normal.y, normal.z);
        uvs.push(...groundUv(x, y, centreX, centreY));
      }
    }
  }
  for (let stripIndex = 0; stripIndex + 1 < stripCount; ++stripIndex) {
    for (let face = 0; face < faces; ++face) {
      const a = (stripIndex * faces + face) * 2;
      const b = a + 1;
      const c = ((stripIndex + 1) * faces + face) * 2;
      const d = c + 1;
      indices.push(a, c, b, b, c, d);
    }
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometry.setAttribute('normal', new THREE.Float32BufferAttribute(normals, 3));
  geometry.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
  geometry.setIndex(indices);
  orientFaces(geometry);
  return geometry;
}

function sleeperGeometry(sleeperLength: number): THREE.BufferGeometry {
  // Box axes: x along track, y across, z down. Narrow the top face to a trapezoid.
  const geometry = new THREE.BoxGeometry(sleeperBottomWidth, sleeperLength, sleeperHeight);
  const position = geometry.getAttribute('position');
  for (let i = 0; i < position.count; ++i) {
    if (position.getZ(i) < 0) {
      position.setX(i, position.getX(i) * (sleeperTopWidth / sleeperBottomWidth));
    }
  }
  geometry.computeVertexNormals();
  return geometry;
}

/** Places instances along the line; offsets are Track-T [lateral, depth] about the centreline at each station. */
function placeAlong(
  model: TrackModel,
  mesh: THREE.InstancedMesh,
  offsetsAt: (stationMeters: number) => [number, number][],
  stationsMeters: number[],
): void {
  const trackFrame = model.trackFrameAtStation(model.firstStationMeters);
  const matrix = new THREE.Matrix4();
  const point = new THREE.Vector3();
  let instance = 0;
  for (const stationMeters of stationsMeters) {
    model.trackFrameAtStation(stationMeters, trackFrame);
    for (const [v, w] of offsetsAt(stationMeters)) {
      point.copy(trackFrame.position).addScaledVector(trackFrame.right, v).addScaledVector(trackFrame.down, w);
      matrix.makeBasis(trackFrame.tangent, trackFrame.right, trackFrame.down).setPosition(point);
      mesh.setMatrixAt(instance++, matrix);
    }
  }
  mesh.instanceMatrix.needsUpdate = true;
  mesh.computeBoundingSphere();
}

const monoFont = 'ui-monospace, "SF Mono", Menlo, "Cascadia Mono", Consolas, "Ubuntu Sans Mono", "Ubuntu Mono", "DejaVu Sans Mono", monospace';
const cjkFont = '"PingFang SC", "Microsoft YaHei", "Noto Sans CJK SC", "Source Han Sans SC", sans-serif';

/** A small sign board drawn on a canvas; world size in metres. */
function signSprite(lines: { text: string; color: string; weight: number; size: number; font: string }[], accent: boolean): THREE.Sprite {
  const scale = 4;
  const width = 176 * scale;
  const height = (18 + lines.length * 26) * scale;
  const canvas = document.createElement('canvas');
  canvas.width = width;
  canvas.height = height;
  const context = canvas.getContext('2d');
  if (context === null) {
    throw new Error('2D canvas is not available');
  }
  context.scale(scale, scale);
  const w = width / scale;
  const h = height / scale;
  context.fillStyle = 'rgba(250, 247, 241, 0.94)';
  context.strokeStyle = accent ? stageColors.accent : 'rgba(43, 36, 24, 0.28)';
  context.lineWidth = accent ? 2 : 1;
  context.beginPath();
  context.roundRect(1, 1, w - 2, h - 2, 6);
  context.fill();
  context.stroke();
  context.fillStyle = accent ? stageColors.accent : stageColors.ink;
  context.fillRect(8, 9, 3, h - 18);
  lines.forEach((line, index) => {
    context.fillStyle = line.color;
    context.font = `${line.weight} ${line.size}px ${line.font}`;
    context.textBaseline = 'middle';
    context.fillText(line.text, 18, 9 + 13 + index * 26);
  });
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.anisotropy = 4;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, transparent: true }));
  const worldWidth = 1.05;
  sprite.scale.set(worldWidth, (worldWidth * h) / w, 1);
  return sprite;
}

export interface BuiltTrack {
  group: THREE.Group;
  ground: THREE.Mesh;
  signs: THREE.Group;
}

export function buildTrack(model: TrackModel, palette: ScenePalette): BuiltTrack {
  const group = new THREE.Group();
  group.name = 'track';
  const halfSpacing = 0.5 * model.medianRailDatumSpacingMeters;

  const loop = railSection();
  const isHead = (w0: number, w1: number): boolean => w0 < 0.0045 && w1 < 0.0045;
  for (const side of ['left', 'right'] as const) {
    const lateralOffsets = side === 'left' ? model.leftRailLateralOffsetsMeters : model.rightRailLateralOffsetsMeters;
    const depthOffsets = side === 'left' ? model.leftRailDepthOffsetsMeters : model.rightRailDepthOffsetsMeters;
    const geometry = extrudeSection(
      model,
      loop,
      (stationIndex) => [lateralOffsets[stationIndex] as number, depthOffsets[stationIndex] as number],
      isHead,
    );
    const rail = new THREE.Mesh(geometry, [palette.railHead, palette.railBody]);
    rail.castShadow = true;
    rail.receiveShadow = true;
    group.add(rail);
  }

  const firstSleeper = Math.ceil(model.firstStationMeters / sleeperSpacing) * sleeperSpacing;
  const sleeperStationsMeters: number[] = [];
  for (let stationMeters = firstSleeper; stationMeters <= model.lastStationMeters; stationMeters += sleeperSpacing) {
    sleeperStationsMeters.push(stationMeters);
  }
  const sleepers = new THREE.InstancedMesh(sleeperGeometry(model.medianRailDatumSpacingMeters + 1.1), palette.sleeper, sleeperStationsMeters.length);
  placeAlong(
    model,
    sleepers,
    (stationMeters) => {
      const [vMid, wMid] = datumMidpoint(model, stationMeters);
      return [[vMid, wMid + railDepth + padThickness + 0.5 * sleeperHeight]];
    },
    sleeperStationsMeters,
  );
  sleepers.castShadow = true;
  sleepers.receiveShadow = true;
  group.add(sleepers);

  const railFeet = (stationMeters: number, depth: number, lateral: number[]): [number, number][] => {
    const result: [number, number][] = [];
    for (const side of ['left', 'right'] as const) {
      const [v, w] = model.railDatumOffsetAtStation(stationMeters, side);
      for (const offset of lateral) {
        result.push([v + offset, w + depth]);
      }
    }
    return result;
  };
  const pads = new THREE.InstancedMesh(new THREE.BoxGeometry(0.18, 0.17, padThickness), palette.fastener, sleeperStationsMeters.length * 2);
  placeAlong(model, pads, (stationMeters) => railFeet(stationMeters, railDepth + 0.5 * padThickness, [0]), sleeperStationsMeters);
  pads.receiveShadow = true;
  group.add(pads);

  const clips = new THREE.InstancedMesh(new THREE.BoxGeometry(0.09, 0.05, 0.034), palette.fastener, sleeperStationsMeters.length * 4);
  placeAlong(model, clips, (stationMeters) => railFeet(stationMeters, railDepth - 0.012, [-0.092, 0.092]), sleeperStationsMeters);
  clips.castShadow = true;
  group.add(clips);

  const ballast = new THREE.Mesh(ballastGeometry(model), palette.ballast);
  ballast.receiveShadow = true;
  group.add(ballast);

  // Ground plane under the lowest point of the line, and the formation strip.
  const centre = model.trackFrameAtStation(0.5 * (model.firstStationMeters + model.lastStationMeters)).position.clone();
  const planeZ = model.lowestCentrelineInertialZMeters + groundDepth;
  const groundGeometry = new THREE.PlaneGeometry(groundSize, groundSize);
  groundGeometry.rotateX(Math.PI);
  const uv = groundGeometry.getAttribute('uv');
  for (let i = 0; i < uv.count; ++i) {
    uv.setXY(i, uv.getX(i) * (groundSize / 10), uv.getY(i) * (groundSize / 10));
  }
  const ground = new THREE.Mesh(groundGeometry, palette.ground);
  ground.position.set(centre.x, centre.y, planeZ);
  ground.receiveShadow = true;
  const formation = new THREE.Mesh(formationGeometry(model, planeZ, halfSpacing + 2.85, centre.x, centre.y), palette.formation);
  formation.receiveShadow = true;
  group.add(formation);

  // Chainage posts every 50 m, and element points in the accent colour. Posts
  // stand on the formation beside the ballast.
  const signs = new THREE.Group();
  signs.name = 'track signs';
  const trackFrame = model.trackFrameAtStation(model.firstStationMeters);
  const lateral = new THREE.Vector3();
  const postGeometry = new THREE.CylinderGeometry(0.03, 0.03, 1.1, 12);
  postGeometry.rotateX(Math.PI / 2);
  const postOffset = halfSpacing + 2.15;
  const addPost = (stationMeters: number, side: -1 | 1, material: THREE.Material, sprite: THREE.Sprite): void => {
    model.trackFrameAtStation(stationMeters, trackFrame);
    horizontalRight(trackFrame, lateral);
    const base = trackFrame.position.clone().addScaledVector(lateral, side * postOffset);
    const baseZ = trackFrame.position.z + groundDepth;
    const post = new THREE.Mesh(postGeometry, material);
    post.position.set(base.x, base.y, baseZ - 0.55);
    post.castShadow = true;
    signs.add(post);
    sprite.position.set(base.x, base.y, baseZ - 1.1 - 0.5 * sprite.scale.y);
    signs.add(sprite);
  };
  const firstPost = Math.ceil(model.firstStationMeters / 50) * 50;
  for (let stationMeters = firstPost; stationMeters <= model.lastStationMeters; stationMeters += 50) {
    const sprite = signSprite([{ text: chainage(stationMeters, 0), color: stageColors.ink, weight: 700, size: 15, font: monoFont }], false);
    addPost(stationMeters, 1, palette.post, sprite);
  }
  const markGeometry = new THREE.BoxGeometry(0.08, model.medianRailDatumSpacingMeters + 1.6, 0.008);
  for (const point of model.elementPoints) {
    const sprite = signSprite(
      [
        { text: `${point.code}  ${point.zh}`, color: stageColors.accent, weight: 700, size: 15, font: `${monoFont}, ${cjkFont}` },
        { text: chainage(point.stationMeters, 1), color: stageColors.ink, weight: 500, size: 13, font: monoFont },
      ],
      true,
    );
    addPost(point.stationMeters, -1, palette.elementMark, sprite);
    model.trackFrameAtStation(point.stationMeters, trackFrame);
    const [vMid, wMid] = datumMidpoint(model, point.stationMeters);
    const mark = new THREE.Mesh(markGeometry, palette.elementMark);
    // A paint line across the sleeper tops, under the rail feet.
    mark.position
      .copy(trackFrame.position)
      .addScaledVector(trackFrame.right, vMid)
      .addScaledVector(trackFrame.down, wMid + railDepth + padThickness - 0.004);
    mark.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(trackFrame.tangent, trackFrame.right, trackFrame.down));
    signs.add(mark);
  }
  group.add(signs);
  return { group, ground, signs };
}
