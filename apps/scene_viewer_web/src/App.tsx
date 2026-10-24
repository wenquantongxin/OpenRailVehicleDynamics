import { useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState } from 'react';

import { loadRecordFromFiles, loadRecordFromUrl } from './record/load_record.ts';
import type { SceneRecord } from './record/scene_record.ts';
import { parseVisualDefinition, type VisualDefinition } from './record/visual_definition.ts';
import { PlaybackController } from './playback/playback.ts';
import type { DisplayDriver, DisplayInterval } from './playback/display_driver.ts';
import type { SceneDefinition } from './record/scene_definition.ts';
import type { SceneSampleView } from './record/scene_sample.ts';
import { LiveConnection, type LiveConnectionStatus } from './live/live_connection.ts';
import { LiveController } from './live/live_controller.ts';
import { LiveSample, resolveLiveSpinBindings } from './live/live_samples.ts';
import { LiveReadoutScales } from './live/live_readout_scales.ts';
import { LiveCard } from './ui/LiveCard.tsx';
import { applyFrames, wheelSpinBindings } from './scene/apply_frame.ts';
import { selectWheelSpin } from './scene/pose_interpolation.ts';
import { summarizeRecord, type RecordSummary } from './ui/record_summary.ts';
import { measureFramedExtents, presetAvailableFor, viewPresets, type FramedExtents, type ViewPreset } from './scene/camera_rig.ts';
import type { SafeArea } from './scene/label_layer.ts';
import { buildScene, type BuiltScene, type CarbodyMode } from './scene/scene_builder.ts';
import { resolveVehicleDisplayBindings, type VehicleDisplayBindings } from './scene/vehicle_display_bindings.ts';
import { EmptyState, Footer, MetaBar, Notice, ScalarsDrawer, TitleBlock } from './ui/Chrome.tsx';
import { recordRangeSubtitle, type Subtitle } from './ui/subtitle.ts';
import { DisplayCard, ViewsCard } from './ui/ControlCards.tsx';
import { fixed } from './ui/format.ts';
import { PlaybackCard, playbackRates } from './ui/PlaybackCard.tsx';
import { buildVehicleReadoutModel, type VehicleReadoutModel } from './ui/vehicle_readout_model.ts';
import { TrackCard } from './ui/TrackCard.tsx';
import { CarriersCard, VehicleCard, WheelForceCard } from './ui/VehicleCards.tsx';
import { measureCardLayout, type EdgeRect } from './viewer/card_layout.ts';
import { SceneViewport, type ViewportCommands, type ViewportOptions } from './viewer/SceneViewport.tsx';

// File and live sources share scene construction, sampled readouts and camera controls.

interface LoadedScene {
  record: SceneRecord | null;
  definition: SceneDefinition;
  driver: DisplayDriver;
  live: { controller: LiveController; scales: LiveReadoutScales; runId: bigint; spins: ReturnType<typeof resolveLiveSpinBindings>; startStation: number } | null;
  visualDefinition: VisualDefinition | null;
  bindings: VehicleDisplayBindings;
  scene: BuiltScene;
  playback: PlaybackController | null;
  readoutModel: VehicleReadoutModel;
  summary: RecordSummary | null;
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
function presetAvailable(preset: ViewPreset, loaded: Pick<LoadedScene, 'bindings' | 'extents' | 'scene'>): boolean {
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

/** Build the common scene once its definition and first display interval are ready. */
function prepareScene(definition: SceneDefinition, initial: SceneSampleView, interval: DisplayInterval) {
  const visualDefinition = definition.visualDefinitionText === null ? null :
    parseVisualDefinition(definition.visualDefinitionText, new Set(definition.bodies.map((body) => body.name)));
  const bindings = resolveVehicleDisplayBindings(definition, visualDefinition);
  const scene = buildScene(definition, visualDefinition, bindings);
  const readoutModel = buildVehicleReadoutModel(definition, scene.track, bindings, initial);
  applyFrames(definition, scene, wheelSpinBindings(definition), interval.first, interval.second, interval.alpha);
  scene.root.updateMatrixWorld(true);
  const extents = measureFramedExtents(definition, scene, bindings);
  scene.setUserHidden(new Set((query.get('hide') ?? '').split(',').filter((part) => part !== '')));
  const wanted = requestedPreset();
  const initialPreset = presetAvailable(wanted, { bindings, extents, scene }) ? wanted : 'overview';
  return { visualDefinition, bindings, scene, readoutModel, extents, initialPreset };
}

export function App() {
  const [loaded, setLoaded] = useState<LoadedScene | null>(null);
  const [loadState, setLoadState] = useState<{ state: 'idle' | 'loading' | 'error'; message: string }>({ state: 'idle', message: '' });
  const loadedRef = useRef<LoadedScene | null>(null);
  const [liveWarning, setLiveWarning] = useState<string | null>(null);
  const [displaySample, setDisplaySample] = useState<SceneSampleView | null>(null);
  const [liveStatus, setLiveStatus] = useState<LiveConnectionStatus>({ state: 'connecting', message: '' });
  const [, refreshLive] = useState(0);
  const liveMode = query.get('live') === '1';
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
    const playback = new PlaybackController(record.samples);
    const startTime = Number(query.get('t'));
    if (query.get('t') !== null && Number.isFinite(startTime)) {
      playback.seek(startTime);
    }
    const initial = playback.interval();
    const prepared = prepareScene(record.definition, record.samples.viewAt(0), initial);
    const summary = summarizeRecord(record.samples, prepared.readoutModel);
    const coarsest = summary.maximumWheelRotationRadians;
    const warning =
      summary.hasMissingWheelSpinAngles && coarsest > Math.PI / 2
        ? `Missing wheel spin angles and up to ${fixed(coarsest, 2)} rad of wheel turn between samples: in-between wheel poses are unreliable. 无轮自转角，插值方向不可靠。`
        : null;
    setLoaded({
      record,
      definition: record.definition,
      driver: playback,
      live: null,
      ...prepared,
      playback,
      summary,
      name,
      subtitle: recordRangeSubtitle(summary.carbodyStation),
      warning,
    });
    setDisplaySample(initial.first);
    setPreset(prepared.initialPreset);
    setFrameIndex(Math.max(0, record.samples.frameIndexAtOrBefore(playback.timeSeconds)));
    setTimeSeconds(playback.timeSeconds);
    setPlaying(false);
    setLoadState({ state: 'idle', message: '' });
    setDefinitionNoticeOpen(true);
  }, []);

  // The deep-linked load is abortable: a superseded run (StrictMode's first
  // effect run in development, or a re-run) neither installs its record nor
  // writes its outcome over the current load state.
  useEffect(() => {
    if (liveMode) {
      if (query.has('record')) setLoadState({ state: 'error', message: 'Choose either live or record. live 与 record 不能同时使用。' });
      return;
    }
    const url = query.get('record');
    if (url === null) {
      if (query.has('bundle')) setLoadState({ state: 'idle', message: 'A resource bundle needs a scene record. Choose a record folder or add a record URL. 资源包需要配合场景记录；请选择记录文件夹或指定 record 地址。' });
      return;
    }
    const controller = new AbortController();
    setLoadState({ state: 'loading', message: '' });
    loadRecordFromUrl(url, controller.signal, query.get('bundle'))
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
  }, [install, liveMode]);

  useEffect(() => {
    if (!liveMode || query.has('record')) return;
    const connection = new LiveConnection({
      ready: (definition, description, message) => {
        const previous = loadedRef.current;
        const live = previous?.live;
        if (live !== null && live !== undefined && live.runId === description.header.runId) {
          const first = new LiveSample(message, live.spins);
          if (!live.controller.reconnect(first)) return;
          live.scales.observe(first);
          const interval = live.controller.interval();
          setDisplaySample(interval.first);
          setTimeSeconds(interval.displayTimeSeconds);
          setPlaying(live.controller.playing);
          setLoadState({ state: 'idle', message: '' });
          return;
        }
        const spins = resolveLiveSpinBindings(definition);
        const first = new LiveSample(message, spins);
        const controller = new LiveController(first);
        const interval = controller.interval();
        const prepared = prepareScene(definition, first, interval);
        const scales = new LiveReadoutScales(prepared.readoutModel);
        scales.observe(first);
        const next: LoadedScene = { record: null, definition, driver: controller,
          live: { controller, scales, runId: description.header.runId, spins,
            startStation: prepared.readoutModel.stationAt(first) ?? Number.NaN },
          ...prepared, playback: null, summary: null,
          name: description.header.runId.toString(), subtitle: null, warning: null };
        loadedRef.current = next;
        setLoaded(next);
        setDisplaySample(interval.first);
        setTimeSeconds(interval.displayTimeSeconds);
        setPlaying(controller.playing);
        setPreset(prepared.initialPreset);
        setLoadState({ state: 'idle', message: '' });
        setDefinitionNoticeOpen(true);
      },
      state: (message) => {
        const current = loadedRef.current;
        if (current?.live === null || current?.live === undefined || current.live.runId !== message.header.runId) return;
        const sample = new LiveSample(message, current.live.spins);
        if (current.live.controller.append(sample)) current.live.scales.observe(sample);
      },
      status: setLiveStatus,
      disconnected: () => loadedRef.current?.live?.controller.disconnect(),
    }, query.get('bundle'));
    connection.start();
    return () => connection.dispose();
  }, [liveMode]);

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

  const onDisplayFrame = useCallback((interval: DisplayInterval, nowPlaying: boolean) => {
    const current = loadedRef.current;
    if (current?.live !== null && current?.live !== undefined &&
      (interval.first.identity.kind !== 'stream' || interval.first.identity.runId !== current.live.runId)) return;
    if (current?.record !== null && current?.record !== undefined) {
      setFrameIndex(Math.max(0, current.record.samples.frameIndexAtOrBefore(interval.first.timeSeconds)));
    }
    setDisplaySample(interval.first);
    setTimeSeconds(interval.displayTimeSeconds);
    setPlaying(nowPlaying);
    if (current?.live !== null && current?.live !== undefined) {
      const dt = interval.second.timeSeconds - interval.first.timeSeconds;
      const missing = [...wheelSpinBindings(current.definition)].some(([bodyIndex, candidates]) =>
        selectWheelSpin(interval.first, interval.second, candidates) === null &&
        Math.hypot(...interval.second.bodyAngularVelocity(bodyIndex)) * dt > Math.PI / 2);
      setLiveWarning(missing ? 'Missing wheel spin angles: in-between wheel poses are unreliable. 无轮自转角，当前区间插值方向不可靠。' : null);
      refreshLive((value) => value + 1);
    }
  }, []);

  useEffect(() => {
    if (loaded?.playback !== null && loaded?.playback !== undefined) {
      loaded.playback.speed = playbackRates[rateIndex] ?? 1;
    }
  }, [loaded, rateIndex]);

  const togglePlay = useCallback(() => {
    if (loaded === null) return;
    if (loaded.live !== null) {
      const driver = loaded.live.controller;
      if (driver.isPaused) driver.resume(); else driver.pause();
      refreshLive((value) => value + 1);
      return;
    }
    if (loaded.playback === null) return;
    loaded.playback.togglePlay();
    setPlaying(loaded.playback.playing);
  }, [loaded]);

  const seek = useCallback(
    (time: number) => {
      if (loaded?.playback === null || loaded?.playback === undefined || loaded.record === null) return;
      loaded.playback.seek(time);
      setTimeSeconds(loaded.playback.timeSeconds);
      setFrameIndex(Math.max(0, loaded.record.samples.frameIndexAtOrBefore(loaded.playback.timeSeconds)));
    },
    [loaded],
  );

  const step = useCallback(
    (direction: -1 | 1) => {
      if (loaded === null || loaded.record === null || loaded.playback === null) return;
      const { record, playback } = loaded;
      playback.playing = false;
      const current = Math.max(0, record.samples.frameIndexAtOrBefore(playback.timeSeconds));
      const onSample = Math.abs(record.samples.timeAt(current) - playback.timeSeconds) < 1e-9;
      const target = direction > 0 ? current + 1 : onSample ? current - 1 : current;
      seek(record.samples.timeAt(Math.min(record.samples.frameCount - 1, Math.max(0, target))));
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
      } else if ((key === 'arrowleft' || key === 'arrowright') && loaded.playback !== null) {
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

  loadedRef.current = loaded;
  useEffect(() => () => loaded?.scene.dispose(), [loaded]);

  const readoutModel = loaded?.readoutModel ?? null;
  const track = loaded?.scene.track ?? null;
  const sample = displaySample;
  const scales = loaded?.live?.scales.scales ?? loaded?.summary?.scales;
  return (
    <div className="app" ref={appRef}>
      <div className="stage">
        {loaded !== null && (
          <SceneViewport
            definition={loaded.definition}
            scene={loaded.scene}
            driver={loaded.driver}
            bindings={loaded.bindings}
            extents={loaded.extents}
            initialPreset={loaded.initialPreset}
            options={optionsRef}
            layout={layoutRef}
            commands={commands}
            stationAt={loaded.readoutModel.stationAt}
            contactPatchesAt={loaded.readoutModel.contactPatchesAt}
            onDisplayFrame={onDisplayFrame}
          />
        )}
      </div>
      <div className="vignette" />
      <div className="hud">
        <TitleBlock definition={loaded?.visualDefinition ?? null} subtitle={loaded?.subtitle ?? null} live={liveMode} />
        <MetaBar
          record={loaded?.record ?? null}
          live={liveMode}
          name={loaded?.name ?? ''}
          intervalSeconds={loaded?.summary?.sampleIntervalSeconds ?? 0}
          onPick={onPick}
        />
        {loaded !== null && readoutModel !== null && sample !== null && scales !== undefined ? (
          <>
            <div className="col left">
              <CarriersCard record={loaded.definition} readoutModel={readoutModel} sample={sample} wheelWidth={loaded.visualDefinition?.wheelVisual.widthMeters ?? null} />
              {track !== null && <TrackCard track={track} readoutModel={readoutModel} sample={sample} startStationMeters={loaded.live?.startStation ?? loaded.summary?.carbodyStation[0] ?? Number.NaN} />}
            </div>
            <div className="col right">
              <VehicleCard record={loaded.definition} readoutModel={readoutModel} bindings={loaded.bindings} sample={sample} scales={scales} />
              <WheelForceCard readoutModel={readoutModel} sample={sample} scales={scales} />
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
            {loaded.record !== null && loaded.summary !== null && <PlaybackCard
              record={loaded.record.samples}
              summary={loaded.summary}
              track={track}
              frameIndex={frameIndex}
              timeSeconds={timeSeconds}
              playing={playing}
              rateIndex={rateIndex}
              onToggle={togglePlay}
              onSeek={seek}
              onStep={step}
              onRate={setRateIndex}
            />}
            {loaded.live !== null && <LiveCard controller={loaded.live.controller} status={liveStatus} sample={sample} timeSeconds={timeSeconds} onToggle={togglePlay} />}
            {(loaded.live !== null ? liveWarning : loaded.warning) !== null && <div className="warning">{loaded.live !== null ? liveWarning : loaded.warning}</div>}
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
                  en="No visual definition: bodies are shown as axes and wheels from their placements"
                  zh="场景没有视觉定义：刚体显示为坐标轴，车轮按描述绘出"
                  onDismiss={() => setDefinitionNoticeOpen(false)}
                />
              )}
            </div>
            {scalarsOpen && <ScalarsDrawer record={loaded.definition} currentSample={sample} onClose={() => setScalarsOpen(false)} />}
          </>
        ) : (
          <EmptyState state={loadState.state} message={loadState.message || liveStatus.message} onPick={onPick} live={liveMode} />
        )}
        <Footer loaded={loaded !== null} live={liveMode} />
      </div>
    </div>
  );
}
