import * as THREE from 'three';

import type { SceneRecord } from '../record/scene_record.ts';
import { parseVisualDefinition, type VisualDefinition, type VisualPart } from '../record/visual_definition.ts';

// Builds the Three.js objects once from the record. Every body is an empty
// Object3D whose pose is written from the frames; parts hang under their body
// in that body's own frame. The root group turns the ORVD inertial frame
// (x forward, y right, z down) into Three.js Y-up: a rotation of +90 degrees
// about x maps (x, y, z)_I to (x, -z, y)_V, so nothing else permutes axes.

export interface ScenePart {
  name: string;
  bodyName: string;
  group: 'vehicle' | 'wheels' | 'track' | 'grid';
  object: THREE.Object3D;
}

export interface BuiltScene {
  root: THREE.Group;
  bodyObjects: Map<string, THREE.Object3D>;
  parts: ScenePart[];
  visualDefinition: VisualDefinition | null;
}

const vehicleMaterial = new THREE.MeshLambertMaterial({ color: 0xe8e2d6 });
const frameMaterial = new THREE.MeshLambertMaterial({ color: 0xc9c2b6 });
const bridgeMaterial = new THREE.MeshLambertMaterial({ color: 0xe0722a });
const barMaterial = new THREE.MeshLambertMaterial({ color: 0x8a8378 });
const wheelMaterial = new THREE.MeshLambertMaterial({ color: 0x2a2a2a });
const rimMarkerMaterial = new THREE.MeshLambertMaterial({ color: 0xe0722a });
const centerlineMaterial = new THREE.LineDashedMaterial({ color: 0x9a9184, dashSize: 1.0, gapSize: 1.0 });
const railMaterial = new THREE.LineBasicMaterial({ color: 0x5b554c });

function materialForBox(bodyName: string): THREE.Material {
  if (bodyName.startsWith('frame')) {
    return frameMaterial;
  }
  if (bodyName.startsWith('axlebridge')) {
    return bridgeMaterial;
  }
  return vehicleMaterial;
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
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, length, 24), material);
  mesh.position.copy(from).add(to).multiplyScalar(0.5);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction.normalize());
  return mesh;
}

function buildPart(part: VisualPart): THREE.Object3D {
  switch (part.kind) {
    case 'box': {
      const mesh = new THREE.Mesh(new THREE.BoxGeometry(...part.sizeMeters), materialForBox(part.bodyName));
      mesh.position.set(...part.centerInBodyFrameMeters);
      return mesh;
    }
    case 'cylinder':
      return cylinderBetween(part.startInBodyFrameMeters, part.endInBodyFrameMeters, part.radiusMeters, barMaterial);
    case 'axes': {
      const axes = new THREE.AxesHelper(part.lengthMeters);
      axes.position.set(...part.originInBodyFrameMeters);
      return axes;
    }
  }
}

function perpendicularTo(axis: THREE.Vector3): THREE.Vector3 {
  const candidate = Math.abs(axis.x) < 0.9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 0, 1);
  return candidate.cross(axis).normalize();
}

function buildWheel(radius: number, width: number, datum: [number, number, number], axis: [number, number, number], rimMarker: boolean): THREE.Object3D {
  const spinAxis = new THREE.Vector3(...axis).normalize();
  const wheel = new THREE.Group();
  const disc = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, width, 40), wheelMaterial);
  disc.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), spinAxis);
  wheel.add(disc);
  if (rimMarker) {
    // A block near the rim, fixed in the wheel body, so the spin that the
    // body's own orientation already carries is visible on a plain disc.
    const marker = new THREE.Mesh(new THREE.BoxGeometry(0.08, width * 1.15, 0.08), rimMarkerMaterial);
    marker.position.copy(perpendicularTo(spinAxis).multiplyScalar(radius * 0.82));
    marker.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), spinAxis);
    wheel.add(marker);
  }
  wheel.position.set(...datum);
  return wheel;
}

function buildTrack(record: SceneRecord): { object: THREE.Group; extent: THREE.Box3 } | null {
  const track = record.track;
  if (track === null) {
    return null;
  }
  const group = new THREE.Group();
  const extent = new THREE.Box3();
  const addLine = (rows: number[][], material: THREE.Material, dashed: boolean): void => {
    const points = rows.map((row) => new THREE.Vector3(row[0] ?? 0, row[1] ?? 0, row[2] ?? 0));
    points.forEach((point) => extent.expandByPoint(point));
    const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), material);
    if (dashed) {
      line.computeLineDistances();
    }
    group.add(line);
  };
  addLine(track.centerlineInInertialMeters, centerlineMaterial, true);
  addLine(track.leftRailDatumInInertialMeters, railMaterial, false);
  addLine(track.rightRailDatumInInertialMeters, railMaterial, false);
  return { object: group, extent };
}

export function buildScene(record: SceneRecord): BuiltScene {
  const root = new THREE.Group();
  root.rotation.x = Math.PI / 2;
  const bodyObjects = new Map<string, THREE.Object3D>();
  const parts: ScenePart[] = [];
  for (const body of record.bodies) {
    const object = new THREE.Object3D();
    object.name = body.name;
    root.add(object);
    bodyObjects.set(body.name, object);
  }

  let visualDefinition: VisualDefinition | null = null;
  if (record.visualDefinitionText !== null) {
    visualDefinition = parseVisualDefinition(record.visualDefinitionText, new Set(bodyObjects.keys()));
    for (const part of visualDefinition.parts) {
      const object = buildPart(part);
      bodyObjects.get(part.bodyName)?.add(object);
      parts.push({ name: part.name, bodyName: part.bodyName, group: 'vehicle', object });
    }
  } else {
    // Without a visual definition every body is shown as its own axes.
    for (const [name, object] of bodyObjects) {
      const axes = new THREE.AxesHelper(0.5);
      object.add(axes);
      parts.push({ name: `${name} axes`, bodyName: name, group: 'vehicle', object: axes });
    }
  }

  for (const placement of record.wheelPlacements) {
    const body = bodyObjects.get(placement.wheelBodyName);
    if (body === undefined) {
      throw new Error(`wheel placement names unknown body '${placement.wheelBodyName}'`);
    }
    const width = visualDefinition?.wheelVisual.widthMeters ?? placement.nominalRollingRadiusMeters * 0.1;
    const wheel = buildWheel(
      placement.nominalRollingRadiusMeters,
      width,
      placement.datumInWheelBodyFrameMeters,
      placement.spinAxisInWheelBodyFrame,
      visualDefinition?.wheelVisual.rimMarker ?? true,
    );
    body.add(wheel);
    parts.push({ name: placement.interfaceName, bodyName: placement.wheelBodyName, group: 'wheels', object: wheel });
  }

  const track = buildTrack(record);
  if (track !== null) {
    root.add(track.object);
    parts.push({ name: 'track datum lines', bodyName: '', group: 'track', object: track.object });
    const center = track.extent.getCenter(new THREE.Vector3());
    const size = track.extent.getSize(new THREE.Vector3());
    const span = Math.max(size.x, size.y) + 40;
    const grid = new THREE.GridHelper(span, Math.max(4, Math.round(span / 5)), 0xd8d0c2, 0xe6dfd3);
    // GridHelper lies in Three's XZ plane; turned into the inertial x-y plane.
    grid.rotation.x = Math.PI / 2;
    grid.position.set(center.x, center.y, 0);
    root.add(grid);
    parts.push({ name: 'ground grid', bodyName: '', group: 'grid', object: grid });
  }

  return { root, bodyObjects, parts, visualDefinition };
}
