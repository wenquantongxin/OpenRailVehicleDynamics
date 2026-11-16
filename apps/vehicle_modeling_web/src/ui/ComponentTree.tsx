import { useEffect, useRef, useState } from 'react';

import { elementKey, sameElement, type ElementReference, type Outline } from '../service/protocol.ts';
import { ElementName } from './ElementName.tsx';
import { identifierLabel } from './identifier_label.ts';
import { PanelHead } from './PanelHead.tsx';
import { treeKinds } from './tree_rows.ts';

// The outline's elements grouped by kind, in the service's order. Kinds are
// collapsed until opened; a filter by name opens every kind it matches, the
// selected element keeps its row whatever the filter says, and a selection
// made elsewhere opens its kind and scrolls to its row.

export function ComponentTree(props: { outline: Outline | null; selection: ElementReference | null; onSelect: (element: ElementReference) => void }) {
  const { outline, selection } = props;
  const [filter, setFilter] = useState('');
  const [openKinds, setOpenKinds] = useState<ReadonlySet<string>>(new Set());
  const list = useRef<HTMLDivElement>(null);

  // A new selection opens its kind and is scrolled to once its row is there.
  const scrollPending = useRef(false);
  useEffect(() => {
    if (selection === null) return;
    scrollPending.current = true;
    setOpenKinds((open) => (open.has(selection.kind) ? open : new Set([...open, selection.kind])));
  }, [selection]);

  useEffect(() => {
    if (selection === null || !scrollPending.current) return;
    const row = list.current?.querySelector(`[data-element="${CSS.escape(elementKey(selection))}"]`);
    if (row === null || row === undefined) return;
    row.scrollIntoView({ block: 'nearest' });
    scrollPending.current = false;
  }, [selection, openKinds]);

  const filtering = filter.trim() !== '';
  const toggle = (kind: string): void =>
    setOpenKinds((open) => {
      const next = new Set(open);
      if (!next.delete(kind)) next.add(kind);
      return next;
    });

  return (
    <nav className="panel tree-panel" aria-label="Component tree">
      <PanelHead en="Components" zh="组件" detail={outline === null ? undefined : `${outline.elements.length}`} />
      <div className="tree-filter">
        <input type="search" placeholder="Filter by name · 按名称筛选" value={filter} spellCheck={false} onChange={(event) => setFilter(event.target.value)} disabled={outline === null} />
      </div>
      <div className="tree" ref={list}>
        {outline === null && (
          <p className="panel-empty">
            Open a vehicle to see its elements. <span className="zh">打开车辆后显示其元素。</span>
          </p>
        )}
        {treeKinds(outline?.elements ?? [], filter, openKinds, selection).map(({ kind, rows, total, matched, open }) => {
          return (
            <section key={kind} className="tree-kind">
              <button type="button" className="kind-row" aria-expanded={open} onClick={() => toggle(kind)}>
                <span className={`chevron ${open ? 'open' : ''}`} aria-hidden="true" />
                <span className="kind-name">{identifierLabel(kind)}</span>
                <span className="kind-count">{filtering ? `${matched} / ${total}` : total}</span>
              </button>
              {open &&
                rows.map((element) => (
                  <button
                    type="button"
                    key={element.name}
                    data-element={elementKey(element)}
                    className={`element-row ${sameElement(element, selection) ? 'selected' : ''}`}
                    aria-current={sameElement(element, selection) || undefined}
                    onClick={() => props.onSelect(element)}
                  >
                    <ElementName name={element.name} />
                  </button>
                ))}
            </section>
          );
        })}
      </div>
    </nav>
  );
}
