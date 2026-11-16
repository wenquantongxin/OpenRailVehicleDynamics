import { useState, type ReactNode } from 'react';

/** A group of secondary messages under a title with their count, closed until opened. */
export function CollapsibleMessageGroup(props: { en: string; zh: string; count: number; children: ReactNode }) {
  const [open, setOpen] = useState(false);
  return (
    <section className="message-group secondary">
      <button type="button" className="group-toggle" aria-expanded={open} onClick={() => setOpen(!open)}>
        <span className={`chevron ${open ? 'open' : ''}`} aria-hidden="true" />
        {props.en} <span className="zh">{props.zh}</span> <span className="count">{props.count}</span>
      </button>
      {open && props.children}
    </section>
  );
}
