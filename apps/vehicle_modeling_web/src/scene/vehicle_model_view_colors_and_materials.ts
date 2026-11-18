import * as THREE from 'three';

import type { Appearance } from '../visual/read_vehicle_visual_definition.ts';

// Every colour of the 3D view, in one place. The view is flat and cool: a
// light grey stage, a quiet grid, geometry in neutral greys of different depth
// for all the visual definition's appearance roles (an appearance names a role,
// not a colour), and colour only where it carries meaning: force elements,
// joints and, strongest of all, the selection. Colours are sRGB, converted by
// three's colour management.

export const viewColors = {
  background: '#f1f3f5',
  /** Neutral white light, so that the greys stay grey; the sky light fades to a cool grey below. */
  light: '#ffffff',
  lightFromBelow: '#d5dbe1',
  gridMinor: '#e4e8ec',
  gridMajor: '#d3d9df',
  /** The +x arrow and its label on the grid. */
  direction: '#5d6976',
  forceElement: '#4b8480',
  joint: '#7767a0',
  bodyMarker: '#87919b',
  selection: '#3776b8',
  axisX: '#c0564a',
  axisY: '#4f9668',
  axisZ: '#4a72b8',
} as const;

/** A translucent shell whose opacity rises towards its silhouette, like an x-ray plate (from the scene viewer). */
export function createXrayMaterial(color: string, base: number, edge: number, power: number): THREE.ShaderMaterial {
  return new THREE.ShaderMaterial({
    uniforms: {
      uColor: { value: new THREE.Color(color) },
      uBase: { value: base },
      uEdge: { value: edge },
      uPower: { value: power },
    },
    vertexShader: /* glsl */ `
      varying vec3 vNormalView;
      varying vec3 vPositionView;
      void main() {
        vec4 viewPosition = modelViewMatrix * vec4(position, 1.0);
        vPositionView = viewPosition.xyz;
        vNormalView = normalize(normalMatrix * normal);
        gl_Position = projectionMatrix * viewPosition;
      }
    `,
    fragmentShader: /* glsl */ `
      uniform vec3 uColor;
      uniform float uBase;
      uniform float uEdge;
      uniform float uPower;
      varying vec3 vNormalView;
      varying vec3 vPositionView;
      void main() {
        vec3 viewDirection = normalize(-vPositionView);
        float facing = abs(dot(normalize(vNormalView), viewDirection));
        float rim = pow(1.0 - facing, uPower);
        gl_FragColor = vec4(uColor, clamp(uBase + uEdge * rim, 0.0, 1.0));
        #include <tonemapping_fragment>
        #include <colorspace_fragment>
      }
    `,
    transparent: true,
    depthWrite: false,
  });
}

/** A matte surface: the view is flat, so nothing shines. */
function matte(color: string, extra: THREE.MeshStandardMaterialParameters = {}): THREE.MeshStandardMaterial {
  return new THREE.MeshStandardMaterial({ color, roughness: 0.85, metalness: 0, ...extra });
}

export interface ModelPalette {
  byAppearance: Record<Appearance, THREE.Material>;
  /** Every solid part when the parts are shown as x-ray. */
  ghost: THREE.Material;
  wheelBody: THREE.Material;
  wheelTread: THREE.Material;
  bodyMarker: THREE.Material;
  forceElement: THREE.Material;
  joint: THREE.Material;
  frameDot: THREE.Material;
  /** Coloured per vertex: x red, y green, z blue. */
  frameAxes: THREE.LineBasicMaterial;
  /** A selected frame's axes, drawn over everything, so that its orientation shows even inside a part. */
  selectedFrameAxes: THREE.LineBasicMaterial;
  /** The drafting outline around a box shell. */
  shellOutline: THREE.LineBasicMaterial;
  selectedPart: THREE.Material;
  selectedTranslucentPart: THREE.Material;
  /** A selected mark is drawn over everything, so it shows even inside a part. */
  selectedMark: THREE.Material;
  dispose: () => void;
}

export function createModelPalette(): ModelPalette {
  const palette: Omit<ModelPalette, 'dispose'> = {
    byAppearance: {
      shell: createXrayMaterial('#8a95a0', 0.03, 0.42, 2.6),
      glass: matte('#7d8894', { transparent: true, opacity: 0.14, depthWrite: false }),
      floor: matte('#e9edf0', { transparent: true, opacity: 0.6, depthWrite: false }),
      structure: matte('#dfe3e7'),
      accent: matte('#a9b3bd'),
      dark: matte('#4d5661'),
    },
    ghost: createXrayMaterial('#6f7a85', 0.035, 0.42, 2.2),
    wheelBody: matte('#5a646f'),
    wheelTread: matte('#8b959f'),
    bodyMarker: matte(viewColors.bodyMarker),
    forceElement: matte(viewColors.forceElement),
    joint: matte(viewColors.joint),
    frameDot: matte('#5d6976'),
    frameAxes: new THREE.LineBasicMaterial({ vertexColors: true }),
    selectedFrameAxes: new THREE.LineBasicMaterial({ vertexColors: true, depthTest: false, depthWrite: false, transparent: true }),
    shellOutline: new THREE.LineBasicMaterial({ color: '#9aa4ae', transparent: true, opacity: 0.85, depthWrite: false }),
    selectedPart: matte(viewColors.selection, { emissive: viewColors.selection, emissiveIntensity: 0.25 }),
    selectedTranslucentPart: createXrayMaterial(viewColors.selection, 0.22, 0.65, 1.6),
    selectedMark: new THREE.MeshBasicMaterial({ color: viewColors.selection, depthTest: false, depthWrite: false, transparent: true }),
  };
  const materials: THREE.Material[] = [
    ...Object.values(palette.byAppearance),
    palette.ghost,
    palette.wheelBody,
    palette.wheelTread,
    palette.bodyMarker,
    palette.forceElement,
    palette.joint,
    palette.frameDot,
    palette.frameAxes,
    palette.selectedFrameAxes,
    palette.shellOutline,
    palette.selectedPart,
    palette.selectedTranslucentPart,
    palette.selectedMark,
  ];
  return { ...palette, dispose: () => materials.forEach((material) => material.dispose()) };
}
