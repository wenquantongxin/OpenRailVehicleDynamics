import * as THREE from 'three';

import { viewColors } from './vehicle_model_view_colors_and_materials.ts';

// A quiet drafting grid on the plane z = 0 of the drawing frame, one line per
// metre and a stronger one every five, a little larger than the drawn vehicle,
// with an arrow beside the vehicle's front end pointing forward, to +x, on the
// side the overview looks from. Built in the drawing frame under a group
// turned like the preview's root.

function gridLines(positions: number[], color: string): THREE.LineSegments {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  return new THREE.LineSegments(geometry, new THREE.LineBasicMaterial({ color }));
}

function forwardLabel(): THREE.Sprite {
  const canvas = document.createElement('canvas');
  canvas.width = 128;
  canvas.height = 64;
  const context = canvas.getContext('2d');
  if (context === null) {
    throw new Error('2D canvas is not available');
  }
  context.fillStyle = viewColors.direction;
  context.font = '600 40px -apple-system, "Segoe UI", "Ubuntu Sans", Ubuntu, Arial, sans-serif';
  context.textAlign = 'center';
  context.textBaseline = 'middle';
  context.fillText('+x', 64, 34);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const label = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, depthWrite: false }));
  label.scale.set(0.8, 0.4, 1);
  return label;
}

/** The grid for a vehicle whose drawing fills the given box of three's world. */
export function buildGroundGrid(bounds: THREE.Box3): THREE.Group {
  const group = new THREE.Group();
  group.rotation.x = Math.PI / 2;
  // World x is the drawing frame's x and world z its y.
  const x0 = Math.floor(bounds.min.x) - 3;
  const x1 = Math.ceil(bounds.max.x) + 3;
  const y0 = Math.floor(bounds.min.z) - 2;
  const y1 = Math.ceil(bounds.max.z) + 2;
  const minor: number[] = [];
  const major: number[] = [];
  for (let x = x0; x <= x1; ++x) (x % 5 === 0 ? major : minor).push(x, y0, 0, x, y1, 0);
  for (let y = y0; y <= y1; ++y) (y % 5 === 0 ? major : minor).push(x0, y, 0, x1, y, 0);
  group.add(gridLines(minor, viewColors.gridMinor), gridLines(major, viewColors.gridMajor));

  // The arrow lies 2 mm above the plane so that it never fights the lines.
  const tip = bounds.max.x - 0.5;
  const side = bounds.min.z - 1;
  const arrowShape = new THREE.Shape()
    .moveTo(tip - 2.4, side - 0.05)
    .lineTo(tip - 0.6, side - 0.05)
    .lineTo(tip - 0.6, side - 0.2)
    .lineTo(tip, side)
    .lineTo(tip - 0.6, side + 0.2)
    .lineTo(tip - 0.6, side + 0.05)
    .lineTo(tip - 2.4, side + 0.05)
    .closePath();
  const arrow = new THREE.Mesh(new THREE.ShapeGeometry(arrowShape), new THREE.MeshBasicMaterial({ color: viewColors.direction, side: THREE.DoubleSide }));
  arrow.position.z = -0.002;
  const label = forwardLabel();
  label.position.set(tip + 0.5, side, -0.25);
  group.add(arrow, label);
  return group;
}

export function disposeGroundGrid(group: THREE.Group): void {
  group.traverse((object) => {
    if (object instanceof THREE.LineSegments || object instanceof THREE.Mesh) {
      object.geometry.dispose();
      (object.material as THREE.Material).dispose();
    }
    if (object instanceof THREE.Sprite) {
      object.material.map?.dispose();
      object.material.dispose();
    }
  });
}
