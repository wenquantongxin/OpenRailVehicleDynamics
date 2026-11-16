import type { ReactNode } from 'react';

/** A docked panel's title strip: English title, Chinese subtitle, and a detail at the right. */
export function PanelHead(props: { en: string; zh: string; detail?: ReactNode }) {
  return (
    <header className="panel-head">
      <span className="en">{props.en}</span>
      <span className="zh">{props.zh}</span>
      {props.detail !== undefined && <span className="detail">{props.detail}</span>}
    </header>
  );
}
