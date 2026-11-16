import { useEffect, useRef, useState } from 'react';

import { elementKey, sameElement, type ElementReference, type Outline } from '../service/vehicle_modeling_service_protocol.ts';
import { DockedPanelTitleStrip } from './DockedPanelTitleStrip.tsx';
import { identifierLabel } from './identifier_display_labels.ts';
import { treeKinds } from './vehicle_element_tree_rules.ts';
import { WrappableElementName } from './WrappableElementName.tsx';

// Every kind the service describes, each with the outline's elements of it in
// the service's order and a button that creates one, also where the vehicle
// has none yet. Kinds are collapsed until opened; a filter by name opens every
// kind it matches, the selected element keeps its row whatever the filter
// says, and a selection made elsewhere opens its kind and scrolls to its row.

export function VehicleElementTree(props: {
  kinds: readonly string[];
  outline: Outline | null;
  selection: ElementReference | null;
  /** The kind being created, if any. */
  creatingKind: string | null;
  onSelect: (element: ElementReference) => void;
  onCreate: (kind: string) => void;
}) {
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
      <DockedPanelTitleStrip en="Components" zh="组件" detail={outline === null ? undefined : `${outline.elements.length}`} />
      <div className="tree-filter">
        <input type="search" placeholder="Filter by name · 按名称筛选" value={filter} spellCheck={false} onChange={(event) => setFilter(event.target.value)} disabled={outline === null} />
      </div>
      <div className="tree" ref={list}>
        {outline === null && (
          <p className="panel-empty">
            Open a vehicle to see its elements. <span className="zh">打开车辆后显示其元素。</span>
          </p>
        )}
        {outline !== null &&
          treeKinds(props.kinds, outline.elements, filter, openKinds, selection).map(({ kind, rows, total, matched, open }) => {
            const label = identifierLabel(kind);
            return (
              <section key={kind} className="tree-kind">
                <div className={`kind-head ${props.creatingKind === kind ? 'creating' : ''}`}>
                  <button type="button" className="kind-row" aria-expanded={open} onClick={() => toggle(kind)}>
                    <span className={`chevron ${open ? 'open' : ''}`} aria-hidden="true" />
                    <span className="kind-name">{label}</span>
                    <span className="kind-count">{filtering ? `${matched} / ${total}` : total}</span>
                  </button>
                  <button type="button" className="create-button" aria-label={`Create a ${label}`} title={`Create a ${label} · 新建`} onClick={() => props.onCreate(kind)}>
                    <span aria-hidden="true">+</span>
                  </button>
                </div>
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
                      <WrappableElementName name={element.name} />
                    </button>
                  ))}
              </section>
            );
          })}
      </div>
    </nav>
  );
}
