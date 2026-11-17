import { useEffect, useRef, useState } from 'react';

import { subjectKey, type CreationOffer, type ElementKindDescription, type ElementReference, type ModelTreeNode, type PropertySubject } from '../service/vehicle_modeling_service_protocol.ts';
import type { OpenedVehicle } from '../session/vehicle_editing_session.ts';
import { DockedPanelTitleStrip } from './DockedPanelTitleStrip.tsx';
import { openRowsCarriedOver, rowsAboveSubject, rowsOpenWhenAVehicleOpens, rowsShownByFilter } from './model_tree_display_rules.ts';
import { ModelTreeRow } from './ModelTreeRow.tsx';

// The left panel: the model tree as the service describes it, by domains at
// the first level. When a vehicle opens, every domain is closed except the
// first that lists elements; what the user opens then stays open through
// edits, undo, redo and every re-read of the tree, a renamed element's rows
// included. A selection made elsewhere, or one whose row moved under another,
// opens the rows above it and is scrolled into view. A filter by name shows
// the element rows it matches with the rows above them, opened.

export function VehicleModelTree(props: {
  tree: ModelTreeNode[] | null;
  /** How many elements the vehicle has. */
  elementCount: number | null;
  kinds: readonly ElementKindDescription[];
  /** A new one for every vehicle opened or resumed. */
  openedVehicle: OpenedVehicle | null;
  selection: PropertySubject | null;
  /** The row whose "+" is in use. */
  creatingFromRow: string | null;
  onSelect: (subject: PropertySubject) => void;
  onCreate: (rowKey: string, offers: CreationOffer[]) => void;
  onAttach: (rowKey: string, bodies: ElementReference[]) => void;
}) {
  const { tree, selection } = props;
  const [filter, setFilter] = useState('');
  const [openRows, setOpenRows] = useState<ReadonlySet<string>>(new Set());
  const list = useRef<HTMLDivElement>(null);

  useEffect(() => {
    setOpenRows(tree === null ? new Set() : rowsOpenWhenAVehicleOpens(tree));
    // Only a vehicle opened or resumed resets what is open.
  }, [props.openedVehicle]);

  // A new reading of the same vehicle's tree carries the open rows over.
  const shown = useRef({ vehicle: props.openedVehicle, tree });
  useEffect(() => {
    const previous = shown.current;
    shown.current = { vehicle: props.openedVehicle, tree };
    const before = previous.tree;
    if (previous.vehicle !== props.openedVehicle || before === null || tree === null || before === tree) return;
    setOpenRows((open) => openRowsCarriedOver(before, tree, open));
  }, [tree, props.openedVehicle]);

  // A selection, or the row it moved under, opens the rows above it; its row is then scrolled into view.
  const above = tree === null || selection === null ? [] : rowsAboveSubject(tree, selection);
  const revealed = selection === null ? '' : `${subjectKey(selection)} under ${above.join(' / ')}`;
  const scrollPending = useRef(false);
  useEffect(() => {
    if (selection === null) return;
    scrollPending.current = true;
    setOpenRows((open) => (above.every((key) => open.has(key)) ? open : new Set([...open, ...above])));
  }, [revealed]);

  useEffect(() => {
    if (selection === null || !scrollPending.current) return;
    const row = list.current?.querySelector(`[data-subject="${CSS.escape(subjectKey(selection))}"]`);
    if (row === null || row === undefined) return;
    row.scrollIntoView({ block: 'nearest' });
    scrollPending.current = false;
  }, [revealed, openRows]);

  const toggle = (key: string): void =>
    setOpenRows((open) => {
      const next = new Set(open);
      if (!next.delete(key)) next.add(key);
      return next;
    });
  const filtered = tree === null || filter.trim() === '' ? null : rowsShownByFilter(tree, filter, selection);
  const surroundings = {
    kinds: props.kinds,
    attachingKinds: props.kinds.flatMap((kind) => kind.attachment?.jointKinds ?? []),
    selection,
    openRows,
    filtered,
    creatingFromRow: props.creatingFromRow,
    onToggle: toggle,
    onSelect: props.onSelect,
    onCreate: props.onCreate,
    onAttach: props.onAttach,
  };

  return (
    <nav className="panel tree-panel" aria-label="Model tree">
      <DockedPanelTitleStrip en="Model tree" zh="模型树" detail={props.elementCount === null ? undefined : `${props.elementCount}`} />
      <div className="tree-filter">
        <input type="search" placeholder="Filter by name · 按名称筛选" value={filter} spellCheck={false} onChange={(event) => setFilter(event.target.value)} disabled={tree === null} />
      </div>
      <div className="tree" ref={list}>
        {tree === null ? (
          <p className="panel-empty">
            Start or open a project to see its model tree. <span className="zh">新建或打开工程后显示其模型树。</span>
          </p>
        ) : (
          tree.map((row) => <ModelTreeRow key={row.key} row={row} parent={null} depth={0} surroundings={surroundings} />)
        )}
      </div>
    </nav>
  );
}
