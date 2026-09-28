import * as THREE from 'three';

import type { Appearance } from '../record/visual_definition.ts';

// One palette for the whole stage: warm paper, matte white structure, a single
// orange accent, graphite running parts and polished steel where wheel meets
// rail. Colours are stated in sRGB and converted by three's colour management.

export const stageColors = {
  background: '#EFE7DA',
  ground: '#E4D9C8',
  groundLine: '#D6C9B5',
  ballast: '#CBBFAE',
  sleeper: '#D9D2C7',
  accent: '#E5611F',
  alert: '#B8321A',
  ink: '#2B2418',
  ok: '#228571',
} as const;

/** A translucent shell whose opacity rises towards its silhouette, like an x-ray plate. */
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

function standard(color: string, roughness: number, metalness: number, extra: THREE.MeshStandardMaterialParameters = {}) {
  return new THREE.MeshStandardMaterial({ color, roughness, metalness, ...extra });
}

function canvasTexture(size: number, paint: (context: CanvasRenderingContext2D, size: number) => void): THREE.CanvasTexture {
  const canvas = document.createElement('canvas');
  canvas.width = size;
  canvas.height = size;
  const context = canvas.getContext('2d');
  if (context === null) {
    throw new Error('2D canvas is not available');
  }
  paint(context, size);
  const texture = new THREE.CanvasTexture(canvas);
  texture.wrapS = THREE.RepeatWrapping;
  texture.wrapT = THREE.RepeatWrapping;
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.anisotropy = 8;
  return texture;
}

/** Deterministic pseudo-random numbers, so every load paints the same gravel. */
function seededRandom(seed: number): () => number {
  let state = seed >>> 0;
  return () => {
    state = (state * 1664525 + 1013904223) >>> 0;
    return state / 4294967296;
  };
}

function ballastTexture(): THREE.CanvasTexture {
  return canvasTexture(256, (context, size) => {
    context.fillStyle = stageColors.ballast;
    context.fillRect(0, 0, size, size);
    const random = seededRandom(7);
    const tones = ['#BDB09D', '#D6CCBD', '#C4B8A6', '#DDD4C7', '#B3A692'];
    for (let stone = 0; stone < 2600; ++stone) {
      context.fillStyle = tones[Math.floor(random() * tones.length)] ?? stageColors.ballast;
      const x = random() * size;
      const y = random() * size;
      const radius = 1.2 + random() * 3.2;
      context.beginPath();
      context.ellipse(x, y, radius, radius * (0.6 + random() * 0.5), random() * Math.PI, 0, 2 * Math.PI);
      context.fill();
    }
  });
}

/** A faint drafting grid: one line per metre, a stronger one every ten metres. */
function groundTexture(): THREE.CanvasTexture {
  return canvasTexture(1024, (context, size) => {
    context.fillStyle = stageColors.ground;
    context.fillRect(0, 0, size, size);
    const step = size / 10;
    for (let line = 0; line <= 10; ++line) {
      const major = line === 0 || line === 10;
      context.strokeStyle = major ? '#D2C4AF' : '#DDD2C1';
      context.lineWidth = major ? 3 : 1.2;
      const offset = line * step;
      context.beginPath();
      context.moveTo(offset, 0);
      context.lineTo(offset, size);
      context.moveTo(0, offset);
      context.lineTo(size, offset);
      context.stroke();
    }
  });
}

export interface ScenePalette {
  byAppearance: Record<Appearance, THREE.Material>;
  /** The carbody shell when drawn opaque. */
  solidShell: THREE.MeshStandardMaterial;
  wheelBody: THREE.MeshStandardMaterial;
  wheelTread: THREE.MeshStandardMaterial;
  wheelTreadTwoPoint: THREE.MeshStandardMaterial;
  wheelTreadLoss: THREE.MeshStandardMaterial;
  /** Contact patch count not ready at this sample: no claim either way. */
  wheelTreadUnknown: THREE.MeshStandardMaterial;
  railBody: THREE.MeshStandardMaterial;
  railHead: THREE.MeshStandardMaterial;
  sleeper: THREE.MeshStandardMaterial;
  fastener: THREE.MeshStandardMaterial;
  ballast: THREE.MeshStandardMaterial;
  ground: THREE.MeshStandardMaterial;
  /** The ground texture on the line-following formation, drawn over a coplanar ground plane. */
  formation: THREE.MeshStandardMaterial;
  post: THREE.MeshStandardMaterial;
  elementMark: THREE.MeshStandardMaterial;
  dispose: () => void;
}

export function createPalette(): ScenePalette {
  const ballastMap = ballastTexture();
  const groundMap = groundTexture();
  const palette: Omit<ScenePalette, 'dispose'> = {
    byAppearance: {
      shell: createXrayMaterial('#FFFFFF', 0.045, 0.55, 2.8),
      glass: standard('#6E665D', 0.12, 0.0, { transparent: true, opacity: 0.1, depthWrite: false }),
      floor: standard('#F7F3EC', 0.8, 0.0, { transparent: true, opacity: 0.55, depthWrite: false }),
      structure: standard('#EEEAE3', 0.7, 0.0),
      accent: standard(stageColors.accent, 0.5, 0.08),
      dark: standard('#3A3531', 0.55, 0.3),
    },
    solidShell: standard('#F2EEE7', 0.55, 0.04),
    wheelBody: standard('#2F2B28', 0.46, 0.5),
    wheelTread: standard('#D3CEC6', 0.26, 0.9),
    wheelTreadTwoPoint: standard(stageColors.accent, 0.35, 0.3, { emissive: stageColors.accent, emissiveIntensity: 0.35 }),
    wheelTreadLoss: standard(stageColors.alert, 0.4, 0.2, { emissive: stageColors.alert, emissiveIntensity: 0.55 }),
    wheelTreadUnknown: standard('#8F8A83', 0.8, 0.0),
    railBody: standard('#9E988F', 0.42, 0.78),
    railHead: standard('#E7E3DD', 0.18, 0.95),
    sleeper: standard(stageColors.sleeper, 0.9, 0.0),
    fastener: standard('#3E3935', 0.6, 0.35),
    ballast: standard('#FFFFFF', 1.0, 0.0, { map: ballastMap }),
    ground: standard('#FFFFFF', 1.0, 0.0, { map: groundMap }),
    formation: standard('#FFFFFF', 1.0, 0.0, { map: groundMap, polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2 }),
    post: standard('#F4F0EA', 0.6, 0.0),
    elementMark: standard(stageColors.accent, 0.6, 0.0, { emissive: stageColors.accent, emissiveIntensity: 0.15 }),
  };
  const materials: THREE.Material[] = [
    ...Object.values(palette.byAppearance),
    palette.solidShell,
    palette.wheelBody,
    palette.wheelTread,
    palette.wheelTreadTwoPoint,
    palette.wheelTreadLoss,
    palette.wheelTreadUnknown,
    palette.railBody,
    palette.railHead,
    palette.sleeper,
    palette.fastener,
    palette.ballast,
    palette.ground,
    palette.formation,
    palette.post,
    palette.elementMark,
  ];
  return {
    ...palette,
    dispose: () => {
      materials.forEach((material) => material.dispose());
      ballastMap.dispose();
      groundMap.dispose();
    },
  };
}
