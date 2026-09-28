import * as THREE from 'three';

import { chainage } from '../ui/format.ts';
import type { ScenePalette } from './materials.ts';
import { stageColors } from './materials.ts';
import type { TrackModel } from './track_model.ts';

// Display geometry for the sampled line. Every section is written in Track-T
// coordinates (lateral v to the right, depth w downwards) about a recorded rail
// datum or the centreline, then placed with the station's recorded origin and
// rotation, so cant is carried by the rotation itself. Rails, pads and clips
// sit on the recorded rail datums; sleepers and ballast are centred between
// them and scale with the recorded gauge. Rail, sleeper and ballast shapes are
// schematic (a 60E1-like rail, B70-like sleepers at 0.6 m); the record carries
// only the design line, without irregularity.
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
  offsetAt: (index: number) => [number, number],
  isHead: (w0: number, w1: number) => boolean,
): THREE.BufferGeometry {
  const stations = Array.from(model.stations);
  const edgeCount = loop.length;
  const orientation = signedArea(loop) > 0 ? 1 : -1;
  const positions: number[] = [];
  const normals: number[] = [];
  const headIndices: number[] = [];
  const bodyIndices: number[] = [];
  const frame = model.frameAt(stations[0] as number);
  const point = new THREE.Vector3();
  const normal = new THREE.Vector3();
  const vertexIndex = (station: number, edge: number, end: number): number => (station * edgeCount + edge) * 2 + end;
  stations.forEach((station, index) => {
    model.frameAt(station, frame);
    const [lateralOffset, depthOffset] = offsetAt(index);
    for (let edge = 0; edge < edgeCount; ++edge) {
      const [v0, w0] = loop[edge] as [number, number];
      const [v1, w1] = loop[(edge + 1) % edgeCount] as [number, number];
      const dv = v1 - v0;
      const dw = w1 - w0;
      const length = Math.hypot(dv, dw);
      const nv = (orientation * dw) / length;
      const nw = (-orientation * dv) / length;
      normal.copy(frame.right).multiplyScalar(nv).addScaledVector(frame.down, nw).normalize();
      for (const [v, w] of [[v0, w0], [v1, w1]] as const) {
        point
          .copy(frame.position)
          .addScaledVector(frame.right, lateralOffset + v)
          .addScaledVector(frame.down, depthOffset + w);
        positions.push(point.x, point.y, point.z);
        normals.push(normal.x, normal.y, normal.z);
      }
    }
  });
  for (let station = 0; station + 1 < stations.length; ++station) {
    for (let edge = 0; edge < edgeCount; ++edge) {
      const a = vertexIndex(station, edge, 0);
      const b = vertexIndex(station, edge, 1);
      const c = vertexIndex(station + 1, edge, 0);
      const d = vertexIndex(station + 1, edge, 1);
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

/** Midpoint between the two recorded rail datums at a station, in Track-T: [v, w]. */
function datumMidpoint(model: TrackModel, station: number): [number, number] {
  const [vl, wl] = model.railOffsetAt(station, 'left');
  const [vr, wr] = model.railOffsetAt(station, 'right');
  return [0.5 * (vl + vr), 0.5 * (wl + wr)];
}

function ballastGeometry(model: TrackModel): THREE.BufferGeometry {
  // Left slope, crib top and right slope, each a strip with its own normals.
  // Widths follow the recorded gauge; depths are below the rail datum level.
  const halfGauge = 0.5 * model.gauge;
  const profile: [number, number][] = [
    [-(halfGauge + 2.2), 1.05],
    [-(halfGauge + 0.95), ballastTopDepth],
    [halfGauge + 0.95, ballastTopDepth],
    [halfGauge + 2.2, 1.05],
  ];
  const positions: number[] = [];
  const normals: number[] = [];
  const uvs: number[] = [];
  const indices: number[] = [];
  const frame = model.frameAt(model.firstStation);
  const point = new THREE.Vector3();
  const normal = new THREE.Vector3();
  const step = 1.0;
  const stationCount = Math.floor((model.lastStation - model.firstStation) / step) + 1;
  const faces = profile.length - 1;
  for (let k = 0; k < stationCount; ++k) {
    const station = Math.min(model.lastStation, model.firstStation + k * step);
    model.frameAt(station, frame);
    const [vMid, wMid] = datumMidpoint(model, station);
    for (let face = 0; face < faces; ++face) {
      const [v0, w0] = profile[face] as [number, number];
      const [v1, w1] = profile[face + 1] as [number, number];
      const dv = v1 - v0;
      const dw = w1 - w0;
      const length = Math.hypot(dv, dw);
      // Outward (upward) normal of a strip running left to right.
      normal.copy(frame.right).multiplyScalar(dw / length).addScaledVector(frame.down, -dv / length).normalize();
      for (const [v, w] of [[v0, w0], [v1, w1]] as const) {
        point.copy(frame.position).addScaledVector(frame.right, vMid + v).addScaledVector(frame.down, wMid + w);
        positions.push(point.x, point.y, point.z);
        normals.push(normal.x, normal.y, normal.z);
        uvs.push(v / 1.2, station / 1.2);
      }
    }
  }
  for (let k = 0; k + 1 < stationCount; ++k) {
    for (let face = 0; face < faces; ++face) {
      const a = (k * faces + face) * 2;
      const b = a + 1;
      const c = ((k + 1) * faces + face) * 2;
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
function horizontalRight(frame: { tangent: THREE.Vector3; right: THREE.Vector3 }, out: THREE.Vector3): THREE.Vector3 {
  out.set(frame.right.x, frame.right.y, 0);
  if (out.lengthSq() < 1e-12) {
    out.set(-frame.tangent.y, frame.tangent.x, 0);
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
  const frame = model.frameAt(model.firstStation);
  const lateral = new THREE.Vector3();
  const normal = new THREE.Vector3();
  const step = 1.0;
  const stationCount = Math.floor((model.lastStation - model.firstStation) / step) + 1;
  const faces = 3;
  for (let k = 0; k < stationCount; ++k) {
    const station = Math.min(model.lastStation, model.firstStation + k * step);
    model.frameAt(station, frame);
    horizontalRight(frame, lateral);
    const topZ = frame.position.z + groundDepth;
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
        const x = frame.position.x + lateral.x * u;
        const y = frame.position.y + lateral.y * u;
        positions.push(x, y, z);
        normals.push(normal.x, normal.y, normal.z);
        uvs.push(...groundUv(x, y, centreX, centreY));
      }
    }
  }
  for (let k = 0; k + 1 < stationCount; ++k) {
    for (let face = 0; face < faces; ++face) {
      const a = (k * faces + face) * 2;
      const b = a + 1;
      const c = ((k + 1) * faces + face) * 2;
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

/** Places instances along the line; offsets are Track-T [v, w] about the centreline at each station. */
function placeAlong(
  model: TrackModel,
  mesh: THREE.InstancedMesh,
  offsetsAt: (station: number) => [number, number][],
  stations: number[],
): void {
  const frame = model.frameAt(model.firstStation);
  const matrix = new THREE.Matrix4();
  const point = new THREE.Vector3();
  let instance = 0;
  for (const station of stations) {
    model.frameAt(station, frame);
    for (const [v, w] of offsetsAt(station)) {
      point.copy(frame.position).addScaledVector(frame.right, v).addScaledVector(frame.down, w);
      matrix.makeBasis(frame.tangent, frame.right, frame.down).setPosition(point);
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
  const halfGauge = 0.5 * model.gauge;

  const loop = railSection();
  const isHead = (w0: number, w1: number): boolean => w0 < 0.0045 && w1 < 0.0045;
  for (const side of ['left', 'right'] as const) {
    const v = side === 'left' ? model.leftRailV : model.rightRailV;
    const w = side === 'left' ? model.leftRailW : model.rightRailW;
    const geometry = extrudeSection(model, loop, (index) => [v[index] as number, w[index] as number], isHead);
    const rail = new THREE.Mesh(geometry, [palette.railHead, palette.railBody]);
    rail.castShadow = true;
    rail.receiveShadow = true;
    group.add(rail);
  }

  const first = Math.ceil(model.firstStation / sleeperSpacing) * sleeperSpacing;
  const sleeperStations: number[] = [];
  for (let station = first; station <= model.lastStation; station += sleeperSpacing) {
    sleeperStations.push(station);
  }
  const sleepers = new THREE.InstancedMesh(sleeperGeometry(model.gauge + 1.1), palette.sleeper, sleeperStations.length);
  placeAlong(
    model,
    sleepers,
    (station) => {
      const [vMid, wMid] = datumMidpoint(model, station);
      return [[vMid, wMid + railDepth + padThickness + 0.5 * sleeperHeight]];
    },
    sleeperStations,
  );
  sleepers.castShadow = true;
  sleepers.receiveShadow = true;
  group.add(sleepers);

  const railFeet = (station: number, depth: number, lateral: number[]): [number, number][] => {
    const result: [number, number][] = [];
    for (const side of ['left', 'right'] as const) {
      const [v, w] = model.railOffsetAt(station, side);
      for (const offset of lateral) {
        result.push([v + offset, w + depth]);
      }
    }
    return result;
  };
  const pads = new THREE.InstancedMesh(new THREE.BoxGeometry(0.18, 0.17, padThickness), palette.fastener, sleeperStations.length * 2);
  placeAlong(model, pads, (station) => railFeet(station, railDepth + 0.5 * padThickness, [0]), sleeperStations);
  pads.receiveShadow = true;
  group.add(pads);

  const clips = new THREE.InstancedMesh(new THREE.BoxGeometry(0.09, 0.05, 0.034), palette.fastener, sleeperStations.length * 4);
  placeAlong(model, clips, (station) => railFeet(station, railDepth - 0.012, [-0.092, 0.092]), sleeperStations);
  clips.castShadow = true;
  group.add(clips);

  const ballast = new THREE.Mesh(ballastGeometry(model), palette.ballast);
  ballast.receiveShadow = true;
  group.add(ballast);

  // Ground plane under the lowest point of the line, and the formation strip.
  const centre = model.frameAt(0.5 * (model.firstStation + model.lastStation)).position.clone();
  const planeZ = model.lowestCentreZ + groundDepth;
  const groundGeometry = new THREE.PlaneGeometry(groundSize, groundSize);
  groundGeometry.rotateX(Math.PI);
  const uv = groundGeometry.getAttribute('uv');
  for (let i = 0; i < uv.count; ++i) {
    uv.setXY(i, uv.getX(i) * (groundSize / 10), uv.getY(i) * (groundSize / 10));
  }
  const ground = new THREE.Mesh(groundGeometry, palette.ground);
  ground.position.set(centre.x, centre.y, planeZ);
  ground.receiveShadow = true;
  const formation = new THREE.Mesh(formationGeometry(model, planeZ, halfGauge + 2.85, centre.x, centre.y), palette.formation);
  formation.receiveShadow = true;
  group.add(formation);

  // Chainage posts every 50 m, and element points in the accent colour. Posts
  // stand on the formation beside the ballast.
  const signs = new THREE.Group();
  signs.name = 'track signs';
  const frame = model.frameAt(model.firstStation);
  const lateral = new THREE.Vector3();
  const postGeometry = new THREE.CylinderGeometry(0.03, 0.03, 1.1, 12);
  postGeometry.rotateX(Math.PI / 2);
  const postOffset = halfGauge + 2.15;
  const addPost = (station: number, side: -1 | 1, material: THREE.Material, sprite: THREE.Sprite): void => {
    model.frameAt(station, frame);
    horizontalRight(frame, lateral);
    const base = frame.position.clone().addScaledVector(lateral, side * postOffset);
    const baseZ = frame.position.z + groundDepth;
    const post = new THREE.Mesh(postGeometry, material);
    post.position.set(base.x, base.y, baseZ - 0.55);
    post.castShadow = true;
    signs.add(post);
    sprite.position.set(base.x, base.y, baseZ - 1.1 - 0.5 * sprite.scale.y);
    signs.add(sprite);
  };
  const firstPost = Math.ceil(model.firstStation / 50) * 50;
  for (let station = firstPost; station <= model.lastStation; station += 50) {
    const sprite = signSprite([{ text: chainage(station, 0), color: stageColors.ink, weight: 700, size: 15, font: monoFont }], false);
    addPost(station, 1, palette.post, sprite);
  }
  const markGeometry = new THREE.BoxGeometry(0.08, model.gauge + 1.6, 0.008);
  for (const point of model.elementPoints) {
    const sprite = signSprite(
      [
        { text: `${point.code}  ${point.zh}`, color: stageColors.accent, weight: 700, size: 15, font: `${monoFont}, ${cjkFont}` },
        { text: chainage(point.station, 1), color: stageColors.ink, weight: 500, size: 13, font: monoFont },
      ],
      true,
    );
    addPost(point.station, -1, palette.elementMark, sprite);
    model.frameAt(point.station, frame);
    const [vMid, wMid] = datumMidpoint(model, point.station);
    const mark = new THREE.Mesh(markGeometry, palette.elementMark);
    // A paint line across the sleeper tops, under the rail feet.
    mark.position
      .copy(frame.position)
      .addScaledVector(frame.right, vMid)
      .addScaledVector(frame.down, wMid + railDepth + padThickness - 0.004);
    mark.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(frame.tangent, frame.right, frame.down));
    signs.add(mark);
  }
  group.add(signs);
  return { group, ground, signs };
}
