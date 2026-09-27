import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

import type { SceneRecord } from '../record/scene_record.ts';
import type { PlaybackController } from '../playback/playback.ts';
import { applyFrames, wheelSpinBindings } from '../scene/apply_frame.ts';
import type { BuiltScene } from '../scene/scene_builder.ts';

export type CameraPreset = 'iso' | 'side' | 'front' | 'top';

export interface ViewportCommands {
  /** Set by the viewport; the panel calls it to move the camera. */
  applyPreset: (preset: CameraPreset) => void;
}

interface Props {
  record: SceneRecord;
  scene: BuiltScene;
  playback: PlaybackController;
  followBodyName: string | null;
  initialPreset: CameraPreset;
  commands: React.MutableRefObject<ViewportCommands | null>;
  onDisplayFrame: (frameA: number, timeSeconds: number, playing: boolean) => void;
}

const presetOffsets: Record<CameraPreset, THREE.Vector3> = {
  iso: new THREE.Vector3(-22, 12, 18),
  side: new THREE.Vector3(0, 3, 26),
  front: new THREE.Vector3(30, 3, 0),
  top: new THREE.Vector3(0.01, 40, 0),
};

export function SceneViewport({ record, scene, playback, followBodyName, initialPreset, commands, onDisplayFrame }: Props) {
  const containerRef = useRef<HTMLDivElement>(null);
  const followRef = useRef(followBodyName);
  followRef.current = followBodyName;

  useEffect(() => {
    const container = containerRef.current;
    if (container === null) {
      return;
    }
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(renderer.domElement);
    const threeScene = new THREE.Scene();
    threeScene.background = new THREE.Color(0xf3ede4);
    threeScene.add(new THREE.HemisphereLight(0xffffff, 0xb8ae9e, 1.1));
    const sun = new THREE.DirectionalLight(0xffffff, 1.4);
    sun.position.set(-30, 60, 40);
    threeScene.add(sun);
    threeScene.add(scene.root);

    const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 4000);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    const target = new THREE.Vector3();
    const followWorld = new THREE.Vector3();

    const followObject = (): THREE.Object3D | undefined =>
      followRef.current === null ? undefined : scene.bodyObjects.get(followRef.current);

    const applyPreset = (preset: CameraPreset): void => {
      const anchor = followObject();
      if (anchor !== undefined) {
        anchor.getWorldPosition(target);
      }
      controls.target.copy(target);
      camera.position.copy(target).add(presetOffsets[preset]);
      controls.update();
    };
    commands.current = { applyPreset };

    const resize = (): void => {
      const width = container.clientWidth;
      const height = container.clientHeight;
      renderer.setSize(width, height, false);
      camera.aspect = width / Math.max(1, height);
      camera.updateProjectionMatrix();
    };
    const observer = new ResizeObserver(resize);
    observer.observe(container);
    resize();

    // Place the initial pose before the first camera preset so the camera
    // starts around the vehicle rather than around the world origin.
    const bindings = wheelSpinBindings(record);
    const initial = playback.bracket();
    applyFrames(record, scene, bindings, initial.frameA, initial.frameB, initial.alpha);
    scene.root.updateMatrixWorld(true);
    applyPreset(initialPreset);

    let previousWall = performance.now();
    let lastReported = -1;
    let lastPlaying = !playback.playing;
    let animationFrame = 0;
    const loop = (): void => {
      animationFrame = requestAnimationFrame(loop);
      const now = performance.now();
      const wallDelta = (now - previousWall) / 1000;
      previousWall = now;
      playback.advance(wallDelta);
      const bracket = playback.bracket();
      applyFrames(record, scene, bindings, bracket.frameA, bracket.frameB, bracket.alpha);
      const anchor = followObject();
      if (anchor !== undefined) {
        scene.root.updateMatrixWorld(true);
        anchor.getWorldPosition(followWorld);
        const shift = followWorld.clone().sub(controls.target);
        controls.target.copy(followWorld);
        camera.position.add(shift);
      }
      controls.update();
      renderer.render(threeScene, camera);
      if (bracket.frameA !== lastReported || playback.playing || playback.playing !== lastPlaying) {
        lastReported = bracket.frameA;
        lastPlaying = playback.playing;
        onDisplayFrame(bracket.frameA, playback.timeSeconds, playback.playing);
      }
    };
    loop();

    return () => {
      cancelAnimationFrame(animationFrame);
      observer.disconnect();
      controls.dispose();
      threeScene.remove(scene.root);
      renderer.dispose();
      container.removeChild(renderer.domElement);
      commands.current = null;
    };
  }, [record, scene, playback, initialPreset, commands, onDisplayFrame]);

  return <div ref={containerRef} className="viewport" />;
}
