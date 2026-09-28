import { useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState } from 'react';

import { loadRecordFromFiles, loadRecordFromUrl } from './record/load_record.ts';
import { frameIndexAtOrBefore, frameTimeSeconds, type SceneRecord } from './record/scene_record.ts';
import { parseVisualDefinition, type VisualDefinition } from './record/visual_definition.ts';
import { PlaybackController } from './playback/playback.ts';
import { applyFrames, wheelSpinBindings } from './scene/apply_frame.ts';
import { maximumWheelRotationBetweenFrames } from './scene/pose_interpolation.ts';
import { measureFramedExtents, presetAvailableFor, viewPresets, type FramedExtents, type ViewPreset } from './scene/camera_rig.ts';
import type { SafeArea } from './scene/label_layer.ts';
import { buildScene, type BuiltScene, type CarbodyMode } from './scene/scene_builder.ts';
import { resolveVehicleDisplayBindings, type VehicleDisplayBindings } from './scene/vehicle_display_bindings.ts';
import { EmptyState, Footer, MetaBar, Notice, ScalarsDrawer, TitleBlock } from './ui/Chrome.tsx';
import { scenarioSubtitle, type Subtitle } from './ui/subtitle.ts';
import { DisplayCard, ViewsCard } from './ui/ControlCards.tsx';
import { fixed } from './ui/format.ts';
import { PlaybackCard, playbackRates } from './ui/PlaybackCard.tsx';
import { buildVehicleReadoutModel, type VehicleReadoutModel } from './ui/vehicle_readout_model.ts';
import { TrackCard } from './ui/TrackCard.tsx';
import { CarriersCard, VehicleCard, WheelForceCard } from './ui/VehicleCards.tsx';
import { measureCardLayout, type EdgeRect } from './viewer/card_layout.ts';
import { SceneViewport, type ViewportCommands, type ViewportOptions } from './viewer/SceneViewport.tsx';

// The replay: load a record, parse its visual definition, resolve the display
// bindings once, build the scene and the readout model from that one
// resolution, measure what the camera presets frame, drive body poses from
// the display clock, and show only what the record carries.

interface LoadedSceneReplay {
  record: SceneRecord;
  visualDefinition: VisualDefinition | null;
  bindings: VehicleDisplayBindings;
  scene: BuiltScene;
  playback: PlaybackController;
  readoutModel: VehicleReadoutModel;
  /** Extents at the first displayed pose; presets and their availability come from these. */
  extents: FramedExtents;
  name: string;
  subtitle: Subtitle | null;
  warning: string | null;
  /** The requested first view, or the overview when the request cannot be framed. */
  initialPreset: ViewPreset;
}

const query = new URLSearchParams(window.location.search);

function initialOptions(): ViewportOptions {
  const carbody = query.get('carbody');
  return {
    follow: query.get('follow') !== '0',
    orbit: query.get('orbit') === '1',
    labels: query.get('labels') === '1',
    carbody: carbody === 'solid' || carbody === 'hidden' ? carbody : 'xray',
    track: query.get('track') !== '0',
    axes: query.get('axes') === '1',
  };
}

function requestedPreset(): ViewPreset {
  const requested = query.get('view');
  return viewPresets.find((preset) => preset === requested) ?? 'overview';
}

/** The one availability rule, shared with the rig: a bound anchor with a measured extent. */
function presetAvailable(preset: ViewPreset, loaded: Pick<LoadedSceneReplay, 'bindings' | 'extents' | 'scene'>): boolean {
  const firstBogie = loaded.bindings.bogies[0];
  const bogieBound = firstBogie !== undefined && loaded.scene.bodyObjects.has(firstBogie.bodyName);
  return presetAvailableFor(preset, loaded.extents, bogieBound);
}

function edgeRect(element: Element | null): EdgeRect | null {
  if (element === null) {
    return null;
  }
  const rect = element.getBoundingClientRect();
  return { left: rect.left, right: rect.right, top: rect.top, bottom: rect.bottom };
}

function recordNameFromUrl(url: string): string {
  const parts = url.split('/').filter((part) => part !== '');
  return parts[parts.length - 1] ?? url;
}

export function App() {
  const [loaded, setLoaded] = useState<LoadedSceneReplay | null>(null);
  const [loadState, setLoadState] = useState<{ state: 'idle' | 'loading' | 'error'; message: string }>({ state: 'idle', message: '' });
  const [frameIndex, setFrameIndex] = useState(0);
  const [timeSeconds, setTimeSeconds] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [rateIndex, setRateIndex] = useState(3);
  const [preset, setPreset] = useState<ViewPreset>(requestedPreset);
  const [options, setOptions] = useState<ViewportOptions>(initialOptions);
  const [scalarsOpen, setScalarsOpen] = useState(false);
  const [definitionNoticeOpen, setDefinitionNoticeOpen] = useState(true);
  const optionsRef = useRef(options);
  optionsRef.current = options;
  const commands = useRef<ViewportCommands | null>(null);
  const appRef = useRef<HTMLDivElement>(null);
  // The stage margins the cards cover, measured from the elements and shared
  // by the camera framing and the label layer; written in place so the
  // viewport keeps one reference and reads it every frame.
  const layoutRef = useRef<SafeArea>({ left: 0, right: 0, top: 0, bottom: 0 });
  const availablePresets = useMemo(
    () => new Set(loaded === null ? viewPresets : viewPresets.filter((candidate) => presetAvailable(candidate, loaded))),
    [loaded],
  );

  useLayoutEffect(() => {
    const app = appRef.current;
    if (app === null || loaded === null) {
      return;
    }
    const stage = app.querySelector('.stage');
    const watched = [stage, app.querySelector('.col.left'), app.querySelector('.col.right'), app.querySelector('.card.playback')];
    const measure = (): void => {
      const stageRect = edgeRect(stage);
      if (stageRect === null) {
        return;
      }
      Object.assign(layoutRef.current, measureCardLayout(stageRect, edgeRect(watched[1] ?? null), edgeRect(watched[2] ?? null), edgeRect(watched[3] ?? null)));
    };
    measure();
    const observer = new ResizeObserver(measure);
    for (const element of watched) {
      if (element !== null) {
        observer.observe(element);
      }
    }
    return () => observer.disconnect();
  }, [loaded]);

  const install = useCallback((record: SceneRecord, name: string) => {
    const visualDefinition =
      record.visualDefinitionText === null
        ? null
        : parseVisualDefinition(record.visualDefinitionText, new Set(record.bodies.map((body) => body.name)));
    const bindings = resolveVehicleDisplayBindings(record, visualDefinition);
    const scene = buildScene(record, visualDefinition, bindings);
    const playback = new PlaybackController(record);
    const readoutModel = buildVehicleReadoutModel(record, scene.track, bindings);
    const startTime = Number(query.get('t'));
    if (query.get('t') !== null && Number.isFinite(startTime)) {
      playback.seek(startTime);
    }
    // Pose the scene at its first displayed time and measure what the camera
    // presets frame, so their availability is known before the viewport exists.
    const initial = playback.bracket();
    applyFrames(record, scene, wheelSpinBindings(record), initial.firstFrameIndex, initial.secondFrameIndex, initial.alpha);
    scene.root.updateMatrixWorld(true);
    const extents = measureFramedExtents(record, scene, bindings);
    // Parts named in `hide` stay hidden whatever the display switches say.
    scene.setUserHidden(new Set((query.get('hide') ?? '').split(',').filter((part) => part !== '')));
    const wheelBodies = record.wheelPlacements
      .map((placement) => record.bodies.findIndex((body) => body.name === placement.wheelBodyName))
      .filter((bodyIndex) => bodyIndex >= 0);
    const coarsest = maximumWheelRotationBetweenFrames(record, wheelBodies);
    const warning =
      record.columns.wheelSpinAngleCount === 0 && coarsest > Math.PI / 2
        ? `No wheel spin angles and up to ${fixed(coarsest, 2)} rad of wheel turn between samples: in-between wheel poses are unreliable. 无轮自转角，插值方向不可靠。`
        : null;
    const wanted = requestedPreset();
    const initialPreset = presetAvailable(wanted, { bindings, extents, scene }) ? wanted : 'overview';
    setLoaded({
      record,
      visualDefinition,
      bindings,
      scene,
      playback,
      readoutModel,
      extents,
      name,
      subtitle: scenarioSubtitle(readoutModel.carbodyStation, readoutModel.initialSpeedKmh, scene.track),
      warning,
      initialPreset,
    });
    setPreset(initialPreset);
    setFrameIndex(Math.max(0, frameIndexAtOrBefore(record, playback.timeSeconds)));
    setTimeSeconds(playback.timeSeconds);
    setPlaying(false);
    setLoadState({ state: 'idle', message: '' });
    setDefinitionNoticeOpen(true);
  }, []);

  // The deep-linked load is abortable: a superseded run (StrictMode's first
  // effect run in development, or a re-run) neither installs its record nor
  // writes its outcome over the current load state.
  useEffect(() => {
    const url = query.get('record');
    if (url === null) {
      return;
    }
    const controller = new AbortController();
    setLoadState({ state: 'loading', message: '' });
    loadRecordFromUrl(url, controller.signal)
      .then((record) => {
        if (!controller.signal.aborted) {
          install(record, recordNameFromUrl(url));
        }
      })
      .catch((error: unknown) => {
        if (!controller.signal.aborted) {
          setLoadState({ state: 'error', message: error instanceof Error ? error.message : String(error) });
        }
      });
    return () => controller.abort();
  }, [install]);

  const onPick = useCallback(
    (files: FileList) => {
      const first = files[0];
      const name = first?.webkitRelativePath.split('/')[0] ?? 'record';
      setLoadState({ state: 'loading', message: '' });
      loadRecordFromFiles(files)
        .then((record) => install(record, name))
        .catch((error: unknown) => setLoadState({ state: 'error', message: error instanceof Error ? error.message : String(error) }));
    },
    [install],
  );

  const onDisplayFrame = useCallback((displayedFrameIndex: number, time: number, nowPlaying: boolean) => {
    setFrameIndex(displayedFrameIndex);
    setTimeSeconds(time);
    setPlaying(nowPlaying);
  }, []);

  useEffect(() => {
    if (loaded !== null) {
      loaded.playback.speed = playbackRates[rateIndex] ?? 1;
    }
  }, [loaded, rateIndex]);

  const togglePlay = useCallback(() => {
    if (loaded === null) {
      return;
    }
    loaded.playback.togglePlay();
    setPlaying(loaded.playback.playing);
  }, [loaded]);

  const seek = useCallback(
    (time: number) => {
      if (loaded === null) {
        return;
      }
      loaded.playback.seek(time);
      setTimeSeconds(loaded.playback.timeSeconds);
      setFrameIndex(Math.max(0, frameIndexAtOrBefore(loaded.record, loaded.playback.timeSeconds)));
    },
    [loaded],
  );

  const step = useCallback(
    (direction: -1 | 1) => {
      if (loaded === null) {
        return;
      }
      const { record, playback } = loaded;
      playback.playing = false;
      const current = Math.max(0, frameIndexAtOrBefore(record, playback.timeSeconds));
      const onSample = Math.abs(frameTimeSeconds(record, current) - playback.timeSeconds) < 1e-9;
      const target = direction > 0 ? current + 1 : onSample ? current - 1 : current;
      seek(frameTimeSeconds(record, Math.min(record.frameCount - 1, Math.max(0, target))));
      setPlaying(false);
    },
    [loaded, seek],
  );

  // Every route to a preset, button, key or deep link, passes this one check.
  const selectPreset = useCallback(
    (next: ViewPreset) => {
      if (loaded === null || !presetAvailable(next, loaded)) {
        return;
      }
      if (commands.current?.applyPreset(next) === true) {
        setPreset(next);
      }
    },
    [loaded],
  );

  const changeOptions = useCallback((next: Partial<ViewportOptions>) => {
    setOptions((previous) => ({ ...previous, ...next }));
  }, []);

  useEffect(() => {
    const onKey = (event: KeyboardEvent): void => {
      const target = event.target as HTMLElement | null;
      if (target !== null && (target.tagName === 'INPUT' || target.tagName === 'TEXTAREA' || target.tagName === 'SELECT')) {
        return;
      }
      if (loaded === null || event.ctrlKey || event.metaKey || event.altKey) {
        return;
      }
      const key = event.key.toLowerCase();
      if (key === ' ') {
        event.preventDefault();
        togglePlay();
      } else if (key === 'arrowleft' || key === 'arrowright') {
        event.preventDefault();
        const direction = key === 'arrowleft' ? -1 : 1;
        if (event.shiftKey) {
          seek(loaded.playback.timeSeconds + direction);
        } else {
          step(direction);
        }
      } else if (/^[1-5]$/.test(key)) {
        const next = viewPresets[Number(key) - 1];
        if (next !== undefined) {
          selectPreset(next);
        }
      } else if (key === 'l') {
        changeOptions({ labels: !optionsRef.current.labels });
      } else if (key === 'x') {
        const order: CarbodyMode[] = ['xray', 'solid', 'hidden'];
        changeOptions({ carbody: order[(order.indexOf(optionsRef.current.carbody) + 1) % order.length] ?? 'xray' });
      } else if (key === 'f') {
        changeOptions({ follow: !optionsRef.current.follow });
      } else if (key === 'o') {
        changeOptions({ orbit: !optionsRef.current.orbit });
      } else if (key === 't') {
        changeOptions({ track: !optionsRef.current.track });
      } else if (key === 'a') {
        changeOptions({ axes: !optionsRef.current.axes });
      } else if (key === 's') {
        setScalarsOpen((open) => !open);
      } else if (key === 'escape') {
        setScalarsOpen(false);
      }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [loaded, togglePlay, seek, step, selectPreset, changeOptions]);

  useEffect(() => () => loaded?.scene.dispose(), [loaded]);

  const readoutModel = loaded?.readoutModel ?? null;
  const track = loaded?.scene.track ?? null;
  return (
    <div className="app" ref={appRef}>
      <div className="stage">
        {loaded !== null && (
          <SceneViewport
            record={loaded.record}
            scene={loaded.scene}
            playback={loaded.playback}
            bindings={loaded.bindings}
            extents={loaded.extents}
            initialPreset={loaded.initialPreset}
            options={optionsRef}
            layout={layoutRef}
            commands={commands}
            contactPatchesAt={loaded.readoutModel.contactPatchesAt}
            onDisplayFrame={onDisplayFrame}
          />
        )}
      </div>
      <div className="vignette" />
      <div className="hud">
        <TitleBlock definition={loaded?.visualDefinition ?? null} subtitle={loaded?.subtitle ?? null} />
        <MetaBar
          record={loaded?.record ?? null}
          name={loaded?.name ?? ''}
          code={loaded?.visualDefinition?.vehicleName ?? ''}
          intervalSeconds={readoutModel?.sampleIntervalSeconds ?? 0}
          onPick={onPick}
        />
        {loaded !== null && readoutModel !== null ? (
          <>
            <div className="col left">
              <CarriersCard record={loaded.record} readoutModel={readoutModel} frameIndex={frameIndex} wheelWidth={loaded.visualDefinition?.wheelVisual.widthMeters ?? null} />
              {track !== null && <TrackCard track={track} readoutModel={readoutModel} frameIndex={frameIndex} />}
            </div>
            <div className="col right">
              <VehicleCard record={loaded.record} readoutModel={readoutModel} bindings={loaded.bindings} frameIndex={frameIndex} />
              <WheelForceCard readoutModel={readoutModel} frameIndex={frameIndex} />
              <ViewsCard
                current={preset}
                available={availablePresets}
                bogieDisplayName={loaded.bindings.bogies[0]?.displayName ?? null}
                onSelect={selectPreset}
                options={options}
                onChange={changeOptions}
              />
              <DisplayCard options={options} onChange={changeOptions} scalarsOpen={scalarsOpen} onToggleScalars={() => setScalarsOpen((open) => !open)} />
            </div>
            <PlaybackCard
              record={loaded.record}
              readoutModel={readoutModel}
              track={track}
              frameIndex={frameIndex}
              timeSeconds={timeSeconds}
              playing={playing}
              rateIndex={rateIndex}
              onToggle={togglePlay}
              onSeek={seek}
              onStep={step}
              onRate={setRateIndex}
            />
            {loaded.warning !== null && <div className="warning">{loaded.warning}</div>}
            <div className="notices">
              {loadState.state === 'loading' && <Notice kind="loading" en="Reading the record…" zh="正在读取记录" />}
              {loadState.state === 'error' && (
                <Notice
                  kind="error"
                  en="Could not open the record"
                  zh="无法打开记录；仍显示原记录"
                  detail={loadState.message}
                  onDismiss={() => setLoadState({ state: 'idle', message: '' })}
                />
              )}
              {loaded.visualDefinition === null && definitionNoticeOpen && (
                <Notice
                  kind="info"
                  en="No visual definition in this record: bodies are shown as axes and wheels from their placements; carbody and bogie readouts and the bogie view are unavailable"
                  zh="记录中没有视觉定义：刚体以坐标轴显示，车轮按记录放置绘出；车体与构架读数、转向架视角不可用"
                  onDismiss={() => setDefinitionNoticeOpen(false)}
                />
              )}
            </div>
            {scalarsOpen && <ScalarsDrawer record={loaded.record} frameIndex={frameIndex} onClose={() => setScalarsOpen(false)} />}
          </>
        ) : (
          <EmptyState state={loadState.state} message={loadState.message} onPick={onPick} />
        )}
        <Footer loaded={loaded !== null} />
      </div>
    </div>
  );
}
