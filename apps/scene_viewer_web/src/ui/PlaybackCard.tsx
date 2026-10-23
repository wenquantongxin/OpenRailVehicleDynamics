import { useLayoutEffect, useMemo, useRef, useState } from 'react';

import type { RecordedSamples } from '../record/recorded_samples.ts';
import { sectionNames, type TrackModel } from '../scene/track_model.ts';
import { Card } from './Card.tsx';
import { chainage, clock, fixed } from './format.ts';
import type { RecordSummary } from './record_summary.ts';
import { elementLabelWidthPixels, layoutElementLabels } from './timeline_labels.ts';

// Card 07. The timeline carries only recorded facts: the curvature under the
// carbody, the line's element points and the samples where a wheel had two
// contact points or none. Seeking and events use frame row numbers; the
// sample identity shown next to the time is the record's own value.

export const playbackRates = [0.1, 0.25, 0.5, 1, 2, 5];

const phaseNames: Record<number, { en: string; zh: string }> = {
  0: { en: 'Initial accepted state', zh: '初始接受态' },
  1: { en: 'Dense intermediate sample', zh: '稠密中间样本' },
  2: { en: 'Accepted endpoint', zh: '接受端点' },
};

interface Props {
  record: RecordedSamples;
  summary: RecordSummary;
  track: TrackModel | null;
  frameIndex: number;
  timeSeconds: number;
  playing: boolean;
  rateIndex: number;
  onToggle: () => void;
  onSeek: (timeSeconds: number) => void;
  onStep: (direction: -1 | 1) => void;
  onRate: (index: number) => void;
}

export function PlaybackCard(props: Props) {
  const { record, summary, track, frameIndex } = props;
  const start = record.timeAt(0);
  const end = record.timeAt(record.frameCount - 1);
  const span = Math.max(1e-9, end - start);
  const fraction = Math.min(1, Math.max(0, (props.timeSeconds - start) / span));

  const strip = useMemo(() => {
    const samples = 480;
    let kmax = 0;
    const values: number[] = [];
    for (let i = 0; i < samples; ++i) {
      const rowIndex = Math.round((i / (samples - 1)) * (record.frameCount - 1));
      const stationMeters = summary.carbodyStation[rowIndex] ?? Number.NaN;
      const k = track !== null && Number.isFinite(stationMeters) ? Math.abs(track.curvatureAtStation(stationMeters)) : 0;
      values.push(k);
      kmax = Math.max(kmax, k);
    }
    const area =
      `M0,34 ` +
      values.map((k, i) => `L${((1000 * i) / (samples - 1)).toFixed(1)},${(34 - (kmax > 0 ? (22 * k) / kmax : 0)).toFixed(2)}`).join(' ') +
      ' L1000,34 Z';
    // Element points: the first frame at which the carbody reaches the point.
    const elements = (track?.elementPoints ?? []).flatMap((point) => {
      for (let rowIndex = 0; rowIndex < record.frameCount; ++rowIndex) {
        if ((summary.carbodyStation[rowIndex] ?? Number.NaN) >= point.stationMeters) {
          return [{ ...point, at: (record.timeAt(rowIndex) - start) / span }];
        }
      }
      return [];
    });
    const events = summary.contactSpans.map((spanEntry) => ({
      at: (record.timeAt(spanEntry.startFrameIndex) - start) / span,
      loss: spanEntry.patches === 0,
    }));
    return { area, elements, events };
  }, [record, summary, track, start, span]);

  const barRef = useRef<HTMLDivElement>(null);
  const [barWidth, setBarWidth] = useState(0);
  useLayoutEffect(() => {
    const bar = barRef.current;
    if (bar === null) return;
    const measure = (): void => setBarWidth(bar.clientWidth);
    measure();
    const observer = new ResizeObserver(measure);
    observer.observe(bar);
    return () => observer.disconnect();
  }, []);
  const labels = useMemo(() => layoutElementLabels(strip.elements, barWidth), [strip.elements, barWidth]);
  const labelledPoints = new Set(labels.map((label) => label.point));
  const seekFromPointer = (clientX: number): void => {
    const bar = barRef.current;
    if (bar === null) {
      return;
    }
    const rect = bar.getBoundingClientRect();
    const f = Math.min(1, Math.max(0, (clientX - rect.left) / Math.max(1, rect.width)));
    props.onSeek(start + f * span);
  };

  const stationMeters = summary.carbodyStation[frameIndex] ?? Number.NaN;
  const section = track !== null && Number.isFinite(stationMeters) ? track.sectionAtStation(stationMeters) : null;
  let current = '';
  if (section !== null && track !== null) {
    const names = sectionNames[section.kind];
    const parts = [`${names.en.toUpperCase()} ${names.zh}`];
    if (section.kind === 'circular' && section.radiusMeters !== null) {
      parts.push(`R ${fixed(section.radiusMeters, 0)} m`);
    }
    if (section.kind !== 'tangent') {
      parts.push(`CANT ${fixed(1000 * track.superelevationAtStation(stationMeters), 0)} mm`);
    }
    current = parts.join(' · ');
  }
  const firstStationMeters = summary.carbodyStation[0] ?? Number.NaN;
  const lastStationMeters = summary.carbodyStation[record.frameCount - 1] ?? Number.NaN;
  const phase = phaseNames[record.viewAt(frameIndex).phase] ?? { en: 'Unknown phase', zh: '未知阶段' };
  const sampleIndex = record.viewAt(frameIndex).sampleIndex;

  return (
    <Card index="07" en="Playback" zh="回放" className="playback">
      <div className="pb-grid">
        <div className="transport">
          <button type="button" className="step" aria-label="Previous sample" title="Previous sample (←)" onClick={() => props.onStep(-1)}>
            <svg viewBox="0 0 12 12" width="12" height="12"><path d="M8.5 2 L3.5 6 L8.5 10" /></svg>
          </button>
          <button type="button" className="play" aria-label={props.playing ? 'Pause' : 'Play'} title="Play or pause (Space)" onClick={props.onToggle}>
            {props.playing ? (
              <svg viewBox="0 0 16 16" width="16" height="16"><rect x="3" y="2" width="3.6" height="12" rx="1" /><rect x="9.4" y="2" width="3.6" height="12" rx="1" /></svg>
            ) : (
              <svg viewBox="0 0 16 16" width="16" height="16"><path d="M4 2.2 L13.6 8 L4 13.8 Z" /></svg>
            )}
          </button>
          <button type="button" className="step" aria-label="Next sample" title="Next sample (→)" onClick={() => props.onStep(1)}>
            <svg viewBox="0 0 12 12" width="12" height="12"><path d="M3.5 2 L8.5 6 L3.5 10" /></svg>
          </button>
        </div>
        <div className="timeline">
          <div className="tl-labels">
            <span>{Number.isFinite(firstStationMeters) ? chainage(firstStationMeters) : clock(0)}</span>
            <span className="current">{current}</span>
            <span>{Number.isFinite(lastStationMeters) ? chainage(lastStationMeters) : clock(span)}</span>
          </div>
          <div
            className="tl-bar"
            ref={barRef}
            onPointerDown={(event) => {
              event.currentTarget.setPointerCapture(event.pointerId);
              seekFromPointer(event.clientX);
            }}
            onPointerMove={(event) => {
              if (event.buttons === 1) {
                seekFromPointer(event.clientX);
              }
            }}
          >
            <span className="rail" />
            <span className="fill" style={{ width: `${100 * fraction}%` }} />
            <div className="tl-strip">
              <svg viewBox="0 0 1000 34" preserveAspectRatio="none" aria-hidden="true">
                <path d={strip.area} className="curvature" />
                {strip.elements.map((point) => (
                  <line key={`${point.code}-${point.stationMeters}`} x1={1000 * point.at} x2={1000 * point.at} y1={12} y2={labelledPoints.has(point) ? 34 : 18} className="element">
                    <title>{point.code} · {chainage(point.stationMeters)}</title>
                  </line>
                ))}
                {strip.events.map((event, index) => (
                  <line key={index} x1={1000 * event.at} x2={1000 * event.at} y1={26} y2={34} className={event.loss ? 'loss' : 'two'} />
                ))}
              </svg>
              {labels.map(({ point, leftPixels }) => (
                <span key={`${point.code}-${point.stationMeters}`} className="element-code" title={`${point.code} · ${chainage(point.stationMeters)}`} style={{ left: leftPixels, width: elementLabelWidthPixels }}>
                  {point.code}
                </span>
              ))}
            </div>
            <span className="knob" style={{ left: `${100 * fraction}%` }} />
          </div>
          <div className="tl-info">
            <span className="time">
              {clock(props.timeSeconds - start)} <i>/ {clock(span)}</i>
            </span>
            <span className="sample">
              {sampleIndex >= 0 ? `Sample #${sampleIndex}` : 'Sample identity absent'} · Frame {frameIndex} · {phase.en} <span className="zh">{phase.zh}</span>
            </span>
            <span>{Number.isFinite(stationMeters) ? chainage(stationMeters) : ''}</span>
          </div>
        </div>
        <div className="rate">
          <div className="rate-head">
            <span className="bi">
              <span className="en">Rate</span>
              <span className="zh">倍速</span>
            </span>
            <b>{playbackRates[props.rateIndex]}×</b>
          </div>
          <input
            type="range"
            min={0}
            max={playbackRates.length - 1}
            step={1}
            value={props.rateIndex}
            aria-label="Playback rate"
            style={{ ['--fill' as string]: `${(100 * props.rateIndex) / (playbackRates.length - 1)}%` }}
            onChange={(event) => props.onRate(Number(event.target.value))}
          />
          <div className="rate-ticks">
            {playbackRates.map((rate) => (
              <span key={rate}>{rate}</span>
            ))}
          </div>
        </div>
      </div>
      <div className="legend">
        <span><i className="swatch curvature" />Curvature 曲率</span>
        <span><i className="swatch two" />Two-point contact 两点接触</span>
        <span><i className="swatch loss" />Loss of contact 轮轨分离</span>
        <span>TS 直缓 · SC 缓圆 · CS 圆缓 · ST 缓直</span>
      </div>
    </Card>
  );
}
