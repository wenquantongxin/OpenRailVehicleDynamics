import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';

import type { AxesPart, VisualPart } from '../visual/read_vehicle_visual_definition.ts';

// Meshes for the visual definition's parts, built in their body's frame as the
// scene viewer builds them (apps/scene_viewer_web/src/scene/scene_builder.ts),
// without shadows and labels.

export type SolidPart = Exclude<VisualPart, AxesPart>;

/** A cylinder whose axis runs from start to end. */
export function cylinderBetween(start: THREE.Vector3, end: THREE.Vector3, radius: number, material: THREE.Material): THREE.Mesh {
  const direction = end.clone().sub(start);
  const length = direction.length();
  if (!(length > 0)) {
    throw new Error('a cylinder needs two distinct end points');
  }
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, length, 28), material);
  mesh.position.copy(start).add(end).multiplyScalar(0.5);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction.normalize());
  return mesh;
}

export function solidPartMesh(part: SolidPart, material: THREE.Material): THREE.Mesh {
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
      mesh = cylinderBetween(new THREE.Vector3(...part.startInBodyFrameMeters), new THREE.Vector3(...part.endInBodyFrameMeters), part.radiusMeters, material);
      break;
    case 'sphere':
      mesh = new THREE.Mesh(new THREE.SphereGeometry(part.radiusMeters, 20, 14), material);
      mesh.position.set(...part.centerInBodyFrameMeters);
      break;
  }
  // Draw the see-through roles after everything else, floor first.
  mesh.renderOrder = { shell: 3, glass: 2, floor: 1, structure: 0, accent: 0, dark: 0 }[part.appearance];
  return mesh;
}

/** Thin outline of a rounded box: both end sections and the four long edges, as in a line drawing (from the scene viewer). */
export function roundedBoxOutline(size: [number, number, number], cornerRadius: number): THREE.BufferGeometry {
  const [length, width, height] = size;
  const radius = Math.min(cornerRadius, 0.5 * Math.min(...size) - 1e-4);
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

/** Three axis lines from the origin, coloured per vertex through the material: x, y, z. */
export function axesLines(lengthMeters: number, colors: readonly [string, string, string], material: THREE.LineBasicMaterial): THREE.LineSegments {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute([0, 0, 0, lengthMeters, 0, 0, 0, 0, 0, 0, lengthMeters, 0, 0, 0, 0, 0, 0, lengthMeters], 3));
  const rgb = colors.flatMap((color) => {
    const linear = new THREE.Color(color);
    return [linear.r, linear.g, linear.b, linear.r, linear.g, linear.b];
  });
  geometry.setAttribute('color', new THREE.Float32BufferAttribute(rgb, 3));
  return new THREE.LineSegments(geometry, material);
}
