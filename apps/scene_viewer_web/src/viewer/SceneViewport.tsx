import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

import type { SceneRecord } from '../record/scene_record.ts';
import type { PlaybackController } from '../playback/playback.ts';
import { applyFrames, wheelSpinBindings } from '../scene/apply_frame.ts';
import { CameraRig, type ViewPreset } from '../scene/camera_rig.ts';
import { LabelLayer, type Rect, type SafeArea } from '../scene/label_layer.ts';
import { stageColors } from '../scene/materials.ts';
import type { BuiltScene, CarbodyMode } from '../scene/scene_builder.ts';
import type { VehicleDisplayBindings } from '../scene/vehicle_display_bindings.ts';

export interface ViewportOptions {
  follow: boolean;
  orbit: boolean;
  labels: boolean;
  carbody: CarbodyMode;
  track: boolean;
  axes: boolean;
}

export interface ViewportCommands {
  /** Set by the viewport; the panel calls it to move the camera. False when the preset's anchor is not bound. */
  applyPreset: (preset: ViewPreset) => boolean;
}

interface Props {
  record: SceneRecord;
  scene: BuiltScene;
  playback: PlaybackController;
  /** Camera anchors and label obstacle groups come from here, never from body names. */
  bindings: VehicleDisplayBindings;
  initialPreset: ViewPreset;
  options: React.MutableRefObject<ViewportOptions>;
  commands: React.MutableRefObject<ViewportCommands | null>;
  /** Fills the loaded contact patch count of every wheel placement at a frame. */
  contactPatchesAt: ((frameIndex: number, out: Float64Array) => void) | null;
  safeArea: SafeArea;
  onDisplayFrame: (frameIndex: number, timeSeconds: number, playing: boolean) => void;
}

/** Portion of the viewport width left between the card columns. */
function freeFraction(width: number, safe: SafeArea): number {
  return width > 0 ? Math.max(0.35, (width - safe.left - safe.right) / width) : 1;
}

export function SceneViewport(props: Props) {
  const { record, scene, playback, bindings, initialPreset, options, commands, contactPatchesAt, safeArea, onDisplayFrame } = props;
  const containerRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const container = containerRef.current;
    if (container === null) {
      return;
    }
    const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
    renderer.setPixelRatio(Math.min(2, window.devicePixelRatio));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.NeutralToneMapping;
    renderer.toneMappingExposure = 1.0;
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFShadowMap;
    container.appendChild(renderer.domElement);

    const threeScene = new THREE.Scene();
    const background = new THREE.Color(stageColors.background);
    threeScene.background = background;
    threeScene.fog = new THREE.Fog(background, 70, 460);
    const environment = new THREE.PMREMGenerator(renderer);
    const environmentMap = environment.fromScene(new RoomEnvironment(), 0.04).texture;
    threeScene.environment = environmentMap;
    threeScene.environmentIntensity = 0.5;
    threeScene.add(new THREE.HemisphereLight(0xfff7ec, 0xd2bea2, 1.0));
    const sun = new THREE.DirectionalLight(0xfff1de, 2.6);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    sun.shadow.camera.left = -17;
    sun.shadow.camera.right = 17;
    sun.shadow.camera.top = 17;
    sun.shadow.camera.bottom = -17;
    sun.shadow.camera.near = 1;
    sun.shadow.camera.far = 140;
    sun.shadow.bias = -0.0004;
    sun.shadow.normalBias = 0.025;
    sun.shadow.radius = 3;
    const sunOffset = new THREE.Vector3(-16, 34, -20);
    threeScene.add(sun, sun.target);
    threeScene.add(scene.root);
    // A sky dome that meets the fog colour at the horizon and lightens towards the zenith.
    const sky = new THREE.Mesh(
      new THREE.SphereGeometry(1800, 32, 16),
      new THREE.ShaderMaterial({
        uniforms: { uHorizon: { value: background.clone() }, uZenith: { value: new THREE.Color('#F7F3EC') } },
        vertexShader: /* glsl */ `
          varying vec3 vDirection;
          void main() {
            vDirection = normalize(position);
            gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
          }
        `,
        fragmentShader: /* glsl */ `
          uniform vec3 uHorizon;
          uniform vec3 uZenith;
          varying vec3 vDirection;
          void main() {
            float height = clamp(vDirection.y, 0.0, 1.0);
            gl_FragColor = vec4(mix(uHorizon, uZenith, pow(height, 0.6)), 1.0);
            #include <tonemapping_fragment>
            #include <colorspace_fragment>
          }
        `,
        side: THREE.BackSide,
        depthWrite: false,
      }),
    );
    sky.renderOrder = -1;
    sky.frustumCulled = false;
    threeScene.add(sky);

    const camera = new THREE.PerspectiveCamera(32, 1, 0.1, 2500);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.minDistance = 1.2;
    controls.maxDistance = 420;
    controls.maxPolarAngle = Math.PI / 2 - 0.01;
    controls.autoRotateSpeed = 0.7;

    // Anchors are the bound bodies; without a bound carbody the rig looks at
    // the mean of all bodies and assumes nothing about any one of them.
    const carbody = bindings.carbody === null ? null : (scene.bodyObjects.get(bindings.carbody.bodyName) ?? null);
    const firstBogie = bindings.bogies[0];
    const bogie = firstBogie === undefined ? null : (scene.bodyObjects.get(firstBogie.bodyName) ?? null);
    const bodies = [...scene.bodyObjects.values()];
    const rig = new CameraRig(camera, controls, { root: scene.root, carbody, bogie, bodies }, scene.track);
    const labels = new LabelLayer(container, scene.labels);

    let width = 1;
    let height = 1;
    const resize = (): void => {
      width = Math.max(1, container.clientWidth);
      height = Math.max(1, container.clientHeight);
      renderer.setSize(width, height, false);
      camera.aspect = width / height;
      // Lift the picture a little so the vehicle sits in the space above the playback card.
      camera.setViewOffset(width, height, 0, Math.round(0.05 * height), width, height);
      camera.updateProjectionMatrix();
    };
    const observer = new ResizeObserver(resize);
    observer.observe(container);
    resize();

    const spinBindings = wheelSpinBindings(record);
    const patches = new Float64Array(record.wheelPlacements.length);
    const applyPose = (firstFrameIndex: number, secondFrameIndex: number, alpha: number): void => {
      applyFrames(record, scene, spinBindings, firstFrameIndex, secondFrameIndex, alpha);
      if (contactPatchesAt !== null) {
        contactPatchesAt(firstFrameIndex, patches);
        scene.setContactStates(patches);
      }
      scene.root.updateMatrixWorld(true);
    };
    const initial = playback.bracket();
    applyPose(initial.firstFrameIndex, initial.secondFrameIndex, initial.alpha);

    // Each bound bogie's running gear, as a box in its frame body's coordinates
    // taken at the first displayed pose from the parts of the bodies its
    // binding groups; projected every frame as a label obstacle.
    const obstacleBoxes = bindings.bogies.flatMap((bound) => {
      const body = scene.bodyObjects.get(bound.bodyName);
      if (body === undefined) {
        return [];
      }
      const memberNames = new Set(bound.memberBodyIndices.map((bodyIndex) => record.bodies[bodyIndex]?.name));
      return [{ body, memberNames, box: new THREE.Box3() }];
    });
    const partBox = new THREE.Box3();
    const corner = new THREE.Vector3();
    for (const part of scene.parts) {
      if (part.group !== 'running_gear' && part.group !== 'wheels') {
        continue;
      }
      const entry = obstacleBoxes.find((candidate) => candidate.memberNames.has(part.bodyName));
      if (entry === undefined) {
        continue;
      }
      partBox.setFromObject(part.object);
      if (partBox.isEmpty()) {
        continue;
      }
      for (let k = 0; k < 8; ++k) {
        corner.set(k & 1 ? partBox.max.x : partBox.min.x, k & 2 ? partBox.max.y : partBox.min.y, k & 4 ? partBox.max.z : partBox.min.z);
        entry.box.expandByPoint(entry.body.worldToLocal(corner));
      }
    }
    const obstacles: Rect[] = obstacleBoxes.map(() => ({ x0: 0, y0: 0, x1: 0, y1: 0 }));
    const projectObstacles = (): void => {
      obstacleBoxes.forEach((entry, index) => {
        const rect = obstacles[index] as Rect;
        rect.x0 = Infinity;
        rect.y0 = Infinity;
        rect.x1 = -Infinity;
        rect.y1 = -Infinity;
        if (entry.box.isEmpty()) {
          return;
        }
        for (let k = 0; k < 8; ++k) {
          corner
            .set(k & 1 ? entry.box.max.x : entry.box.min.x, k & 2 ? entry.box.max.y : entry.box.min.y, k & 4 ? entry.box.max.z : entry.box.min.z)
            .applyMatrix4(entry.body.matrixWorld)
            .project(camera);
          const x = ((corner.x + 1) / 2) * width;
          const y = ((1 - corner.y) / 2) * height;
          rect.x0 = Math.min(rect.x0, x);
          rect.y0 = Math.min(rect.y0, y);
          rect.x1 = Math.max(rect.x1, x);
          rect.y1 = Math.max(rect.y1, y);
        }
      });
    };
    rig.applyPreset(initialPreset, freeFraction(width, safeArea), true);
    commands.current = {
      applyPreset: (preset) => rig.applyPreset(preset, freeFraction(width, safeArea), false),
    };

    let applied: ViewportOptions | null = null;
    const applyOptions = (): void => {
      const next = options.current;
      if (applied !== null && applied.carbody === next.carbody && applied.track === next.track && applied.axes === next.axes && applied.labels === next.labels) {
        rig.follow = next.follow;
        controls.autoRotate = next.orbit;
        return;
      }
      scene.setDisplay({ carbody: next.carbody, track: next.track, axes: next.axes });
      labels.setVisible(next.labels);
      rig.follow = next.follow;
      controls.autoRotate = next.orbit;
      applied = { ...next };
    };

    // The sun follows the bound carbody, or the mean of all bodies without one.
    const sunAnchor = new THREE.Vector3();
    const bodyWorld = new THREE.Vector3();
    const updateSun = (): void => {
      if (carbody !== null) {
        carbody.getWorldPosition(sunAnchor);
      } else if (bodies.length > 0) {
        sunAnchor.set(0, 0, 0);
        for (const body of bodies) {
          sunAnchor.add(body.getWorldPosition(bodyWorld));
        }
        sunAnchor.multiplyScalar(1 / bodies.length);
      } else {
        return;
      }
      sun.target.position.copy(sunAnchor);
      sun.position.copy(sunAnchor).add(sunOffset);
      sun.target.updateMatrixWorld();
    };
    let previousWall = performance.now();
    let lastReportWall = 0;
    let lastReportedFrameIndex = -1;
    let lastReportedPlaying = !playback.playing;
    let lastReportedTime = Number.NaN;
    let animationFrame = 0;
    const loop = (): void => {
      animationFrame = requestAnimationFrame(loop);
      const now = performance.now();
      const wallDelta = Math.min(0.1, (now - previousWall) / 1000);
      previousWall = now;
      applyOptions();
      playback.advance(wallDelta);
      const bracket = playback.bracket();
      applyPose(bracket.firstFrameIndex, bracket.secondFrameIndex, bracket.alpha);
      rig.update(wallDelta, freeFraction(width, safeArea));
      updateSun();
      sky.position.copy(camera.position);
      renderer.render(threeScene, camera);
      if (options.current.labels) {
        projectObstacles();
      }
      labels.update(camera, width, height, safeArea, obstacles);
      const changed =
        bracket.firstFrameIndex !== lastReportedFrameIndex || playback.playing !== lastReportedPlaying || playback.timeSeconds !== lastReportedTime;
      if (changed && (!playback.playing || now - lastReportWall > 32 || playback.playing !== lastReportedPlaying)) {
        lastReportWall = now;
        lastReportedFrameIndex = bracket.firstFrameIndex;
        lastReportedPlaying = playback.playing;
        lastReportedTime = playback.timeSeconds;
        onDisplayFrame(bracket.firstFrameIndex, playback.timeSeconds, playback.playing);
      }
    };
    loop();

    return () => {
      cancelAnimationFrame(animationFrame);
      observer.disconnect();
      controls.dispose();
      labels.dispose();
      threeScene.remove(scene.root);
      environmentMap.dispose();
      sky.geometry.dispose();
      sky.material.dispose();
      environment.dispose();
      renderer.dispose();
      renderer.domElement.remove();
      commands.current = null;
    };
  }, [record, scene, playback, bindings, initialPreset, options, commands, contactPatchesAt, safeArea, onDisplayFrame]);

  return <div ref={containerRef} className="viewport" />;
}
