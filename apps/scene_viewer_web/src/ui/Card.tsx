import type { ReactNode } from 'react';

// Shared card shell: a numbered header with an English title and its Chinese
// subtitle, in the order the reference uses (accent bar, title, index).

export function Card(props: { index: string; en: string; zh: string; className?: string; children: ReactNode }) {
  return (
    <section className={`card ${props.className ?? ''}`}>
      <header className="card-head">
        <span className="bar" />
        <span className="en">{props.en}</span>
        <span className="zh">{props.zh}</span>
        <span className="idx">{props.index}</span>
      </header>
      {props.children}
    </section>
  );
}

/** An English term with its Chinese subtitle. */
export function Bi(props: { en: string; zh: string; unit?: string; className?: string; title?: string }) {
  return (
    <span className={`bi ${props.className ?? ''}`} title={props.title}>
      <span className="en">
        {props.en}
        {props.unit !== undefined && <span className="unit"> {props.unit}</span>}
      </span>
      <span className="zh">{props.zh}</span>
    </span>
  );
}

/** One readout row: a bilingual key and a value that is either a number or a state. */
export function ReadoutRow(props: { en: string; zh: string; value: ReactNode; muted?: boolean }) {
  return (
    <div className="kv">
      <Bi en={props.en} zh={props.zh} className="k" />
      <span className={`v ${props.muted === true ? 'muted' : ''}`}>{props.value}</span>
    </div>
  );
}

/** A centred bar for signed values, filled from the middle towards the value. */
export function DivergingBar(props: { value: number | null; scale: number }) {
  const fraction = props.value === null ? 0 : Math.max(-1, Math.min(1, props.value / props.scale)) * 50;
  return (
    <span className="dv">
      <i style={{ left: `${50 + Math.min(0, fraction)}%`, width: `${Math.abs(fraction)}%` }} />
      <em />
    </span>
  );
}
