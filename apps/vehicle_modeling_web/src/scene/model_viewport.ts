import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

import type { ElementReference, Preview } from '../service/protocol.ts';
import type { VisualDefinition } from '../visual/visual_definition.ts';
import { presetPlacement, type ViewPreset } from './camera_presets.ts';
import { buildGroundGrid, disposeGroundGrid } from './ground_grid.ts';
import { createModelPalette, viewColors } from './materials.ts';
import { drawsAnything, PreviewScene, type Drawing, type SceneDisplay } from './preview_scene.ts';

// The 3D view: renderer, orbit camera, lights, the ground grid and the drawn
// preview, with picking. It renders only when something changed: a camera
// move, a new drawing, a highlight or a display switch.

/** A press and release closer than this, in pixels, is a click; anything longer moved the camera. */
const clickTolerancePixels = 4;

export interface ViewportCallbacks {
  /** A click hit this drawing, or nothing. */
  pick: (drawing: Drawing | null) => void;
  /** The text shown beside the pointer over a drawing. */
  describe: (drawing: Drawing) => string;
}

export class ModelViewport {
  private readonly container: HTMLElement;
  private readonly callbacks: ViewportCallbacks;
  private readonly renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera(30, 1, 0.05, 2000);
  private readonly controls: OrbitControls;
  private readonly palette = createModelPalette();
  private readonly environment: THREE.PMREMGenerator;
  private readonly environmentMap: THREE.Texture;
  private readonly resizeObserver: ResizeObserver;
  private readonly hoverLabel = document.createElement('div');
  private readonly raycaster = new THREE.Raycaster();
  private preview: PreviewScene | null = null;
  private grid: THREE.Group | null = null;
  private display: SceneDisplay = { parts: 'solid', frames: false };
  private highlighted: ElementReference | null = null;
  private needsRender = true;
  private animationFrame = 0;
  private pressedAt: { x: number; y: number } | null = null;
  private hoverAt: { x: number; y: number } | null = null;

  constructor(container: HTMLElement, callbacks: ViewportCallbacks) {
    this.container = container;
    this.callbacks = callbacks;
    this.renderer.setPixelRatio(Math.min(2, window.devicePixelRatio));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.NeutralToneMapping;
    container.appendChild(this.renderer.domElement);
    this.hoverLabel.className = 'hover-label';
    container.appendChild(this.hoverLabel);

    this.scene.background = new THREE.Color(viewColors.background);
    this.environment = new THREE.PMREMGenerator(this.renderer);
    this.environmentMap = this.environment.fromScene(new RoomEnvironment(), 0.04).texture;
    this.scene.environment = this.environmentMap;
    this.scene.environmentIntensity = 0.5;
    const sun = new THREE.DirectionalLight(viewColors.light, 1.9);
    sun.position.set(-12, 30, -18);
    this.scene.add(new THREE.HemisphereLight(viewColors.light, viewColors.lightFromBelow, 1.1), sun);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.12;
    this.controls.zoomToCursor = true;
    this.controls.minDistance = 0.3;
    this.controls.maxDistance = 400;
    // Every camera move: the gliding of a drag and also a wheel zoom, which the controls apply inside the wheel event.
    this.controls.addEventListener('change', () => {
      this.needsRender = true;
    });
    this.camera.position.set(14, 8, -14);

    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
    this.resize();

    const canvas = this.renderer.domElement;
    // three rebuilds its GPU state on the next render after a lost context comes back.
    canvas.addEventListener('webglcontextrestored', () => {
      this.needsRender = true;
    });
    canvas.addEventListener('pointerdown', this.onPointerDown);
    canvas.addEventListener('pointerup', this.onPointerUp);
    canvas.addEventListener('pointermove', this.onPointerMove);
    canvas.addEventListener('pointerleave', this.onPointerLeave);
    this.loop();
  }

  /**
   * Replaces the drawing. Fitting also lays a new ground grid under it and
   * frames it in the overview; without anything drawn there is no grid.
   */
  showPreview(preview: Preview | null, visualDefinition: VisualDefinition | null, fit: boolean): void {
    if (this.preview !== null) {
      this.scene.remove(this.preview.root);
      this.preview.dispose();
      this.preview = null;
    }
    if (preview !== null) {
      const drawn = new PreviewScene(preview, visualDefinition, this.palette);
      drawn.setDisplay(this.display);
      drawn.setHighlight(this.highlighted);
      this.scene.add(drawn.root);
      this.preview = drawn;
    }
    if (this.grid !== null && (fit || preview === null || !drawsAnything(preview))) {
      this.scene.remove(this.grid);
      disposeGroundGrid(this.grid);
      this.grid = null;
    }
    if (fit && this.preview !== null) {
      this.grid = buildGroundGrid(this.preview.bounds());
      this.scene.add(this.grid);
      this.applyPreset('overview');
    }
    this.needsRender = true;
  }

  setDisplay(display: SceneDisplay): void {
    this.display = display;
    this.preview?.setDisplay(display);
    this.needsRender = true;
  }

  setHighlight(element: ElementReference | null): void {
    this.highlighted = element;
    this.preview?.setHighlight(element);
    this.needsRender = true;
  }

  applyPreset(preset: ViewPreset): void {
    if (this.preview === null) return;
    const placement = presetPlacement(preset, this.preview.bounds(), this.camera.aspect);
    this.camera.position.copy(placement.position);
    this.camera.fov = placement.fov;
    this.camera.updateProjectionMatrix();
    this.controls.target.copy(placement.target);
    this.controls.update();
    this.needsRender = true;
  }

  dispose(): void {
    cancelAnimationFrame(this.animationFrame);
    this.resizeObserver.disconnect();
    this.controls.dispose();
    this.preview?.dispose();
    if (this.grid !== null) disposeGroundGrid(this.grid);
    this.palette.dispose();
    this.environmentMap.dispose();
    this.environment.dispose();
    this.renderer.dispose();
    this.renderer.domElement.remove();
    this.hoverLabel.remove();
  }

  private resize(): void {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.needsRender = true;
  }

  private loop = (): void => {
    this.animationFrame = requestAnimationFrame(this.loop);
    if (this.hoverAt !== null) {
      this.showHover(this.hoverAt);
      this.hoverAt = null;
    }
    // With damping the camera keeps gliding for a while after a drag; each step is a change.
    this.controls.update();
    if (this.needsRender) {
      this.preview?.fitMarksToScreen(this.camera, this.renderer.domElement.clientHeight);
      this.renderer.render(this.scene, this.camera);
      this.needsRender = false;
    }
  };

  private drawingAt(clientX: number, clientY: number): Drawing | null {
    if (this.preview === null) return null;
    const rect = this.renderer.domElement.getBoundingClientRect();
    const pointer = new THREE.Vector2(((clientX - rect.left) / rect.width) * 2 - 1, -((clientY - rect.top) / rect.height) * 2 + 1);
    this.raycaster.setFromCamera(pointer, this.camera);
    return this.preview.pick(this.raycaster);
  }

  private showHover(at: { x: number; y: number }): void {
    const drawing = this.drawingAt(at.x, at.y);
    this.renderer.domElement.style.cursor = drawing === null ? '' : 'pointer';
    if (drawing === null) {
      this.hoverLabel.style.display = 'none';
      return;
    }
    const rect = this.container.getBoundingClientRect();
    this.hoverLabel.textContent = this.callbacks.describe(drawing);
    this.hoverLabel.style.display = 'block';
    // Beside the pointer, on its left where the right would run out of the view.
    const x = at.x - rect.left;
    const left = x + 14 + this.hoverLabel.offsetWidth > rect.width ? x - 14 - this.hoverLabel.offsetWidth : x + 14;
    this.hoverLabel.style.left = `${left}px`;
    this.hoverLabel.style.top = `${at.y - rect.top + 14}px`;
  }

  private onPointerDown = (event: PointerEvent): void => {
    this.pressedAt = event.button === 0 ? { x: event.clientX, y: event.clientY } : null;
    this.hoverLabel.style.display = 'none';
  };

  private onPointerUp = (event: PointerEvent): void => {
    const pressedAt = this.pressedAt;
    this.pressedAt = null;
    if (pressedAt === null || Math.hypot(event.clientX - pressedAt.x, event.clientY - pressedAt.y) > clickTolerancePixels) return;
    this.callbacks.pick(this.drawingAt(event.clientX, event.clientY));
  };

  private onPointerMove = (event: PointerEvent): void => {
    // Hover is resolved once per frame, not once per pointer event.
    this.hoverAt = event.buttons === 0 ? { x: event.clientX, y: event.clientY } : null;
  };

  private onPointerLeave = (): void => {
    this.hoverAt = null;
    this.hoverLabel.style.display = 'none';
  };
}
