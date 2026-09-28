import { useRef } from 'react';

import { ScalarStatus, scalarSample, type SceneRecord } from '../record/scene_record.ts';
import type { VisualDefinition } from '../record/visual_definition.ts';
import { Card } from './Card.tsx';
import { clock, fixed } from './format.ts';
import type { Subtitle } from './subtitle.ts';

// Title block, record meta, footer, the load screen and the scalar drawer.

export function TitleBlock({ definition, subtitle }: { definition: VisualDefinition | null; subtitle: Subtitle | null }) {
  const code = definition?.vehicleName ?? 'ORVD';
  return (
    <div className="brand">
      <div className="tile">
        <span className="slashes">
          <i />
          <i />
          <i />
        </span>
        <span>{code}</span>
      </div>
      <div className="titles">
        <div className="overline">
          ORVD · Scene record replay <span className="zh">场景记录回放</span>
        </div>
        <h1 className="title">
          {definition?.displayName.en ?? 'Scene viewer'}
          <span className="slash">/</span>
          <span className="zh">{definition?.displayName.zh ?? '场景查看器'}</span>
        </h1>
        {subtitle !== null && (
          <div className="subtitle">
            {subtitle.en} <span className="zh">{subtitle.zh}</span>
          </div>
        )}
      </div>
    </div>
  );
}

function FolderButton({ onPick, en, zh, className }: { onPick: (files: FileList) => void; en: string; zh: string; className?: string }) {
  const input = useRef<HTMLInputElement>(null);
  return (
    <>
      <button type="button" className={className ?? 'chip'} onClick={() => input.current?.click()}>
        <span className="en">{en}</span>
        <span className="zh">{zh}</span>
      </button>
      <input
        ref={input}
        type="file"
        hidden
        // @ts-expect-error webkitdirectory is a non-standard attribute understood by browsers
        webkitdirectory=""
        multiple
        onChange={(event) => {
          if (event.target.files !== null && event.target.files.length > 0) {
            onPick(event.target.files);
          }
          event.target.value = '';
        }}
      />
    </>
  );
}

export function MetaBar(props: {
  record: SceneRecord | null;
  name: string;
  code: string;
  intervalSeconds: number;
  onPick: (files: FileList) => void;
}) {
  const { record } = props;
  const duration = record === null ? 0 : (record.timesSeconds[record.frameCount - 1] ?? 0) - (record.timesSeconds[0] ?? 0);
  return (
    <div className="meta-bar">
      {record !== null && (
        <>
          <span className="meta">
            <span className="k">Record</span>
            <span className="v">{props.name}</span>
          </span>
          <span className="meta">
            <span className="k">Frames</span>
            <span className="v">{record.frameCount}</span>
          </span>
          <span className="meta">
            <span className="k">Δt</span>
            <span className="v">{fixed(1000 * props.intervalSeconds, props.intervalSeconds < 0.01 ? 1 : 0)} ms</span>
          </span>
          <span className="meta">
            <span className="k">Span</span>
            <span className="v">{clock(duration)}</span>
          </span>
          <span className="hatch">//////</span>
          <span className="code">{props.code}</span>
        </>
      )}
      <FolderButton onPick={props.onPick} en="Open record" zh="打开记录" className="chip open" />
    </div>
  );
}

export function Footer({ loaded }: { loaded: boolean }) {
  return (
    <footer className="footer">
      <span>
        ORVD scene record · parametric schematic model · readouts are recorded samples
        <span className="zh">参数化示意模型，读数为记录样本</span>
      </span>
      {loaded && <span className="keys">Space play · ← → sample · 1–5 views · L labels · X carbody</span>}
      <span>
        Schematic dimensions, not as-built <span className="zh">示意尺寸</span> <b>////</b>
      </span>
    </footer>
  );
}

/** A notice over the stage: a load in progress, a failed load, or missing record content. */
export function Notice(props: { kind: 'loading' | 'error' | 'info'; en: string; zh: string; detail?: string; onDismiss?: () => void }) {
  return (
    <div className={`notice ${props.kind}`} role={props.kind === 'error' ? 'alert' : 'status'}>
      <span className="notice-mark" />
      <span className="notice-text">
        <span className="en">{props.en}</span>
        <span className="zh">{props.zh}</span>
        {props.detail !== undefined && <span className="detail">{props.detail}</span>}
      </span>
      {props.onDismiss !== undefined && (
        <button type="button" className="notice-close" aria-label="Dismiss" onClick={props.onDismiss}>
          ×
        </button>
      )}
    </div>
  );
}

export function EmptyState(props: { state: 'idle' | 'loading' | 'error'; message: string; onPick: (files: FileList) => void }) {
  return (
    <div className="empty">
      <Card index="00" en="Scene record" zh="场景记录">
        {props.state === 'loading' ? (
          <p className="empty-text">
            Reading the record… <span className="zh">正在读取记录</span>
          </p>
        ) : (
          <>
            <p className="empty-text">
              Open a folder written by <code>ORVD::scene_record</code>: <code>scene.json</code>, <code>frames.f64le</code>,{' '}
              <code>scalar_statuses.u8</code> and <code>visual_definition.json</code>. A served record opens with{' '}
              <code>?record=/records/&lt;name&gt;/</code>.
              <span className="zh">选择由 ORVD 场景记录写出的文件夹，或用 record 参数打开已部署的记录。</span>
            </p>
            {props.state === 'error' && <p className="empty-error">{props.message}</p>}
            <FolderButton onPick={props.onPick} en="Choose record folder" zh="选择记录文件夹" className="chip on" />
          </>
        )}
      </Card>
    </div>
  );
}

export function ScalarsDrawer({ record, frameIndex, onClose }: { record: SceneRecord; frameIndex: number; onClose: () => void }) {
  const groups = new Map<string, number[]>();
  record.scalars.forEach((definition, index) => {
    const owner = definition.name.includes('.') ? definition.name.slice(0, definition.name.indexOf('.')) : '';
    groups.set(owner, [...(groups.get(owner) ?? []), index]);
  });
  return (
    <aside className="drawer card">
      <header className="card-head">
        <span className="bar" />
        <span className="en">All recorded scalars</span>
        <span className="zh">全部标量</span>
        <button type="button" className="close" aria-label="Close" onClick={onClose}>
          ×
        </button>
      </header>
      <p className="caption">
        Raw values in SI units at the displayed sample; hover a name for its quantity, frame and method.
        <span className="zh">当前样本的原始值（国际单位），悬停查看物理量、参考系与方法。</span>
      </p>
      {[...groups.entries()].map(([owner, indices]) => (
        <section key={owner} className="drawer-group">
          <h3>{owner === '' ? 'Record' : owner}</h3>
          {indices.map((index) => {
            const definition = record.scalars[index];
            if (definition === undefined) {
              return null;
            }
            const sample = scalarSample(record, frameIndex, index);
            const status =
              sample.status === ScalarStatus.valid ? null : sample.status === ScalarStatus.placeholder ? 'Placeholder 占位' : 'Not ready 未就绪';
            return (
              <div className="drawer-row" key={definition.name} title={`${definition.quantity}\n${definition.referenceFrame}\n${definition.method}`}>
                <span className="name">{definition.name.slice(owner.length + (owner === '' ? 0 : 1))}</span>
                <b>{status ?? `${Number(sample.value.toPrecision(6))} ${definition.unit === '1' ? '' : definition.unit}`}</b>
              </div>
            );
          })}
        </section>
      ))}
    </aside>
  );
}
