import { useCallback, useEffect, useMemo, useRef, useState } from 'react';

import { loadRecordFromFiles, loadRecordFromUrl } from './record/load_record.ts';
import { frameAtOrBefore, frameTimeSeconds, type SceneRecord } from './record/scene_record.ts';
import { PlaybackController } from './playback/playback.ts';
import { maximumWheelRotationBetweenFrames } from './scene/pose_interpolation.ts';
import { viewPresets, type ViewPreset } from './scene/camera_rig.ts';
import type { SafeArea } from './scene/label_layer.ts';
import { buildScene, type BuiltScene, type CarbodyMode } from './scene/scene_builder.ts';
import { EmptyState, Footer, MetaBar, Notice, ScalarsDrawer, TitleBlock } from './ui/Chrome.tsx';
import { scenarioSubtitle, type Subtitle } from './ui/subtitle.ts';
import { DisplayCard, ViewsCard } from './ui/ControlCards.tsx';
import { fixed } from './ui/format.ts';
import { PlaybackCard, playbackRates } from './ui/PlaybackCard.tsx';
import { buildReadoutModel, type ReadoutModel } from './ui/readout_model.ts';
import { TrackCard } from './ui/TrackCard.tsx';
import { AxleBridgeCard, VehicleCard, WheelForceCard } from './ui/VehicleCards.tsx';
import { SceneViewport, type ViewportCommands, type ViewportOptions } from './viewer/SceneViewport.tsx';

// The replay: load a record, build the scene once, drive body poses from the
// display clock, and show only what the record carries.

interface Loaded {
  record: SceneRecord;
  scene: BuiltScene;
  playback: PlaybackController;
  model: ReadoutModel;
  name: string;
  subtitle: Subtitle | null;
  warning: string | null;
}

/** Card columns and the playback card, which labels and framing keep clear of. */
const safeArea: SafeArea = { left: 294, right: 294, top: 86, bottom: 176 };

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

function initialPreset(): ViewPreset {
  const requested = query.get('view');
  return viewPresets.find((preset) => preset === requested) ?? 'overview';
}

function recordNameFromUrl(url: string): string {
  const parts = url.split('/').filter((part) => part !== '');
  return parts[parts.length - 1] ?? url;
}

export function App() {
  const [loaded, setLoaded] = useState<Loaded | null>(null);
  const [loadState, setLoadState] = useState<{ state: 'idle' | 'loading' | 'error'; message: string }>({ state: 'idle', message: '' });
  const [frame, setFrame] = useState(0);
  const [timeSeconds, setTimeSeconds] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [rateIndex, setRateIndex] = useState(3);
  const [preset, setPreset] = useState<ViewPreset>(initialPreset);
  const [options, setOptions] = useState<ViewportOptions>(initialOptions);
  const [scalarsOpen, setScalarsOpen] = useState(false);
  const [definitionNoticeOpen, setDefinitionNoticeOpen] = useState(true);
  const optionsRef = useRef(options);
  optionsRef.current = options;
  const commands = useRef<ViewportCommands | null>(null);
  const firstPreset = useMemo(initialPreset, []);
  const bogieBodyNames = useMemo(() => loaded?.model.bogies.map((bogie) => bogie.bodyName) ?? [], [loaded]);

  const install = useCallback((record: SceneRecord, name: string) => {
    const scene = buildScene(record);
    const playback = new PlaybackController(record);
    const model = buildReadoutModel(record, scene.track);
    const startTime = Number(query.get('t'));
    if (query.get('t') !== null && Number.isFinite(startTime)) {
      playback.seek(startTime);
    }
    // Parts named in `hide` stay hidden whatever the display switches say.
    scene.setUserHidden(new Set((query.get('hide') ?? '').split(',').filter((part) => part !== '')));
    const wheelBodies = record.wheelPlacements
      .map((placement) => record.bodies.findIndex((body) => body.name === placement.wheelBodyName))
      .filter((index) => index >= 0);
    const coarsest = maximumWheelRotationBetweenFrames(record, wheelBodies);
    const warning =
      record.columns.wheelSpinCount === 0 && coarsest > Math.PI / 2
        ? `No wheel spin angles and up to ${fixed(coarsest, 2)} rad of wheel turn between samples: in-between wheel poses are unreliable. 无轮自转角，插值方向不可靠。`
        : null;
    setLoaded({ record, scene, playback, model, name, subtitle: scenarioSubtitle(model.carbodyStation, model.initialSpeedKmh, scene.track), warning });
    setFrame(Math.max(0, frameAtOrBefore(record, playback.timeSeconds)));
    setTimeSeconds(playback.timeSeconds);
    setPlaying(false);
    setLoadState({ state: 'idle', message: '' });
    setDefinitionNoticeOpen(true);
  }, []);

  useEffect(() => {
    const url = query.get('record');
    if (url === null) {
      return;
    }
    setLoadState({ state: 'loading', message: '' });
    loadRecordFromUrl(url)
      .then((record) => install(record, recordNameFromUrl(url)))
      .catch((error: unknown) => setLoadState({ state: 'error', message: error instanceof Error ? error.message : String(error) }));
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

  const onDisplayFrame = useCallback((displayed: number, time: number, nowPlaying: boolean) => {
    setFrame(displayed);
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
      setFrame(Math.max(0, frameAtOrBefore(loaded.record, loaded.playback.timeSeconds)));
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
      const current = Math.max(0, frameAtOrBefore(record, playback.timeSeconds));
      const onSample = Math.abs(frameTimeSeconds(record, current) - playback.timeSeconds) < 1e-9;
      const target = direction > 0 ? current + 1 : onSample ? current - 1 : current;
      seek(frameTimeSeconds(record, Math.min(record.frameCount - 1, Math.max(0, target))));
      setPlaying(false);
    },
    [loaded, seek],
  );

  const selectPreset = useCallback((next: ViewPreset) => {
    commands.current?.applyPreset(next);
    setPreset(next);
  }, []);

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

  const model = loaded?.model ?? null;
  const track = loaded?.scene.track ?? null;
  return (
    <div className="app">
      <div className="stage">
        {loaded !== null && (
          <SceneViewport
            record={loaded.record}
            scene={loaded.scene}
            playback={loaded.playback}
            carbodyBodyName={loaded.model.carbody?.bodyName ?? null}
            bogieBodyName={loaded.model.bogies[0]?.bodyName ?? null}
            bogieBodyNames={bogieBodyNames}
            initialPreset={firstPreset}
            options={optionsRef}
            commands={commands}
            contactPatchesAt={loaded.model.contactPatchesAt}
            safeArea={safeArea}
            onDisplayFrame={onDisplayFrame}
          />
        )}
      </div>
      <div className="vignette" />
      <div className="hud">
        <TitleBlock definition={loaded?.scene.visualDefinition ?? null} subtitle={loaded?.subtitle ?? null} />
        <MetaBar
          record={loaded?.record ?? null}
          name={loaded?.name ?? ''}
          code={loaded?.scene.visualDefinition?.vehicleName ?? ''}
          intervalSeconds={model?.sampleIntervalSeconds ?? 0}
          onPick={onPick}
        />
        {loaded !== null && model !== null ? (
          <>
            <div className="col left">
              <AxleBridgeCard record={loaded.record} model={model} frame={frame} wheelWidth={loaded.scene.visualDefinition?.wheelVisual.widthMeters ?? null} />
              {track !== null && <TrackCard track={track} model={model} frame={frame} />}
            </div>
            <div className="col right">
              <VehicleCard record={loaded.record} model={model} frame={frame} />
              <WheelForceCard model={model} frame={frame} />
              <ViewsCard current={preset} onSelect={selectPreset} options={options} onChange={changeOptions} />
              <DisplayCard options={options} onChange={changeOptions} scalarsOpen={scalarsOpen} onToggleScalars={() => setScalarsOpen((open) => !open)} />
            </div>
            <PlaybackCard
              record={loaded.record}
              model={model}
              track={track}
              frame={frame}
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
              {loaded.scene.visualDefinition === null && definitionNoticeOpen && (
                <Notice
                  kind="info"
                  en="No visual definition in this record: bodies are shown as axes"
                  zh="记录中没有视觉定义：刚体以坐标轴显示，车轮按记录放置绘出"
                  onDismiss={() => setDefinitionNoticeOpen(false)}
                />
              )}
            </div>
            {scalarsOpen && <ScalarsDrawer record={loaded.record} frame={frame} onClose={() => setScalarsOpen(false)} />}
          </>
        ) : (
          <EmptyState state={loadState.state} message={loadState.message} onPick={onPick} />
        )}
        <Footer loaded={loaded !== null} />
      </div>
    </div>
  );
}
