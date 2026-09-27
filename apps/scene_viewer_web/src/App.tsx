import { useCallback, useEffect, useMemo, useRef, useState } from 'react';

import { loadRecordFromFiles, loadRecordFromUrl } from './record/load_record.ts';
import {
  SamplePhaseNames,
  ScalarStatus,
  framePhase,
  frameSampleIndex,
  scalarSample,
  type SceneRecord,
} from './record/scene_record.ts';
import { PlaybackController } from './playback/playback.ts';
import { maximumWheelRotationBetweenFrames } from './scene/pose_interpolation.ts';
import { buildScene, type BuiltScene } from './scene/scene_builder.ts';
import { SceneViewport, type CameraPreset, type ViewportCommands } from './viewer/SceneViewport.tsx';

// The plain replay: load a record, build the scene once, then drive body
// poses from the display clock. Readouts show only what the record carries.

interface Loaded {
  record: SceneRecord;
  scene: BuiltScene;
  playback: PlaybackController;
}

const speeds = [0.1, 0.25, 0.5, 1, 2, 5];

function statusText(status: number): string {
  if (status === ScalarStatus.notReady) {
    return '未就绪';
  }
  if (status === ScalarStatus.placeholder) {
    return '占位';
  }
  return '';
}

export function App() {
  const [loaded, setLoaded] = useState<Loaded | null>(null);
  const [status, setStatus] = useState('等待记录');
  const [displayFrame, setDisplayFrame] = useState(0);
  const [displayTime, setDisplayTime] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [follow, setFollow] = useState(true);
  const [hidden, setHidden] = useState<Set<string>>(new Set());
  const commands = useRef<ViewportCommands | null>(null);

  const install = useCallback((record: SceneRecord) => {
    const scene = buildScene(record);
    const playback = new PlaybackController(record);
    // Deep links: `hide` names parts to start hidden, `t` a start time.
    const query = new URLSearchParams(window.location.search);
    const hiddenAtStart = new Set((query.get('hide') ?? '').split(',').filter((name) => name !== ''));
    const startTime = Number(query.get('t'));
    if (Number.isFinite(startTime) && query.get('t') !== null) {
      playback.seek(startTime);
    }
    setHidden(hiddenAtStart);
    setLoaded({ record, scene, playback });
    setDisplayFrame(0);
    setDisplayTime(playback.timeSeconds);
    setPlaying(false);
    const wheelBodies = record.wheelPlacements
      .map((placement) => record.bodies.findIndex((body) => body.name === placement.wheelBodyName))
      .filter((index) => index >= 0);
    const coarsest = maximumWheelRotationBetweenFrames(record, wheelBodies);
    const spinNote =
      record.columns.wheelSpinCount > 0
        ? '，带未折返轮自转角'
        : coarsest > Math.PI / 2
          ? `，警告：无轮自转角且相邻样本间轮转角达 ${coarsest.toFixed(2)} rad，轮子插值方向不可靠`
          : '';
    setStatus(
      `记录已加载：${record.frameCount} 帧，${record.bodies.length} 个刚体，` +
        `${record.wheelPlacements.length} 个车轮，${record.scalars.length} 个标量，` +
        `时间 ${playback.startSeconds.toFixed(3)} s 到 ${playback.endSeconds.toFixed(3)} s` +
        (record.track === null ? '，无线路' : `，线路 ${record.track.stationsMeters.length} 站`) +
        spinNote,
    );
  }, []);

  useEffect(() => {
    const url = new URLSearchParams(window.location.search).get('record');
    if (url === null) {
      return;
    }
    setStatus(`正在读取 ${url}`);
    loadRecordFromUrl(url)
      .then(install)
      .catch((error: unknown) => setStatus(`读取失败：${error instanceof Error ? error.message : String(error)}`));
  }, [install]);

  const onPickFolder = (files: FileList | null): void => {
    if (files === null || files.length === 0) {
      return;
    }
    setStatus('正在读取所选文件夹');
    loadRecordFromFiles(files)
      .then(install)
      .catch((error: unknown) => setStatus(`读取失败：${error instanceof Error ? error.message : String(error)}`));
  };

  const onDisplayFrame = useCallback((frame: number, time: number, nowPlaying: boolean) => {
    setDisplayFrame(frame);
    setDisplayTime(time);
    setPlaying(nowPlaying);
  }, []);

  useEffect(() => {
    if (loaded === null) {
      return;
    }
    loaded.playback.speed = speed;
  }, [loaded, speed]);

  const togglePlay = (): void => {
    if (loaded === null) {
      return;
    }
    loaded.playback.togglePlay();
    setPlaying(loaded.playback.playing);
  };

  useEffect(() => {
    if (loaded === null) {
      return;
    }
    for (const part of loaded.scene.parts) {
      part.object.visible = !hidden.has(part.name);
    }
  }, [loaded, hidden]);

  const followBodyName = useMemo(() => {
    if (loaded === null || !follow) {
      return null;
    }
    const carbody = loaded.record.bodies.find((body) => body.name === 'carbody');
    return (carbody ?? loaded.record.bodies[0])?.name ?? null;
  }, [loaded, follow]);

  const togglePart = (name: string): void => {
    setHidden((previous) => {
      const next = new Set(previous);
      if (next.has(name)) {
        next.delete(name);
      } else {
        next.add(name);
      }
      return next;
    });
  };

  const toggleGroup = (group: string, visible: boolean): void => {
    if (loaded === null) {
      return;
    }
    setHidden((previous) => {
      const next = new Set(previous);
      for (const part of loaded.scene.parts) {
        if (part.group === group) {
          if (visible) {
            next.delete(part.name);
          } else {
            next.add(part.name);
          }
        }
      }
      return next;
    });
  };

  const seek = (time: number): void => {
    if (loaded === null) {
      return;
    }
    loaded.playback.seek(time);
    setDisplayTime(loaded.playback.timeSeconds);
  };

  const preset = (name: CameraPreset): void => commands.current?.applyPreset(name);
  const initialPreset = useMemo((): CameraPreset => {
    const requested = new URLSearchParams(window.location.search).get('view');
    return requested === 'side' || requested === 'front' || requested === 'top' ? requested : 'iso';
  }, []);

  return (
    <div className="layout">
      <div className="stage">
        {loaded !== null && (
          <SceneViewport
            record={loaded.record}
            scene={loaded.scene}
            playback={loaded.playback}
            followBodyName={followBodyName}
            initialPreset={initialPreset}
            commands={commands}
            onDisplayFrame={onDisplayFrame}
          />
        )}
      </div>
      <aside className="panel">
        <h1>ORVD 场景回放</h1>
        <p id="viewer-status" className="status">
          {status}
        </p>
        <label className="picker">
          选择记录文件夹
          <input
            type="file"
            // @ts-expect-error webkitdirectory is a non-standard attribute understood by browsers
            webkitdirectory=""
            multiple
            onChange={(event) => onPickFolder(event.target.files)}
          />
        </label>
        {loaded !== null && (
          <>
            <section>
              <h2>播放</h2>
              <div className="row">
                <button type="button" onClick={togglePlay}>
                  {playing ? '暂停' : '播放'}
                </button>
                <select value={speed} onChange={(event) => setSpeed(Number(event.target.value))}>
                  {speeds.map((value) => (
                    <option key={value} value={value}>
                      {value}×
                    </option>
                  ))}
                </select>
                <label>
                  <input type="checkbox" checked={follow} onChange={(event) => setFollow(event.target.checked)} />
                  跟随车体
                </label>
              </div>
              <input
                type="range"
                min={loaded.playback.startSeconds}
                max={loaded.playback.endSeconds}
                step={Math.max(1e-4, (loaded.playback.endSeconds - loaded.playback.startSeconds) / 2000)}
                value={displayTime}
                onChange={(event) => seek(Number(event.target.value))}
              />
              <div className="row">
                <button type="button" onClick={() => preset('iso')}>
                  斜视
                </button>
                <button type="button" onClick={() => preset('side')}>
                  侧视
                </button>
                <button type="button" onClick={() => preset('front')}>
                  正视
                </button>
                <button type="button" onClick={() => preset('top')}>
                  俯视
                </button>
              </div>
            </section>
            <section>
              <h2>读数</h2>
              <table className="readouts">
                <tbody>
                  <tr>
                    <td>显示时间</td>
                    <td>{displayTime.toFixed(4)} s</td>
                  </tr>
                  <tr>
                    <td>当前样本</td>
                    <td>
                      #{frameSampleIndex(loaded.record, displayFrame)} · {SamplePhaseNames[framePhase(loaded.record, displayFrame)] ?? '未知阶段'}
                    </td>
                  </tr>
                  {loaded.record.scalars.map((definition, index) => {
                    const sample = scalarSample(loaded.record, displayFrame, index);
                    const text = statusText(sample.status);
                    return (
                      <tr key={definition.name}>
                        <td title={`${definition.quantity}; ${definition.referenceFrame}; ${definition.method}`}>
                          {definition.name}
                        </td>
                        <td>{text !== '' ? text : `${sample.value.toPrecision(6)} ${definition.unit}`}</td>
                      </tr>
                    );
                  })}
                </tbody>
              </table>
            </section>
            <section>
              <h2>显隐</h2>
              {(['vehicle', 'wheels', 'track', 'grid'] as const).map((group) => (
                <div key={group} className="row">
                  <button type="button" onClick={() => toggleGroup(group, true)}>
                    全显 {group}
                  </button>
                  <button type="button" onClick={() => toggleGroup(group, false)}>
                    全隐 {group}
                  </button>
                </div>
              ))}
              <div className="parts">
                {loaded.scene.parts.map((part) => (
                  <label key={part.name}>
                    <input type="checkbox" checked={!hidden.has(part.name)} onChange={() => togglePart(part.name)} />
                    {part.name}
                  </label>
                ))}
              </div>
            </section>
          </>
        )}
      </aside>
    </div>
  );
}
