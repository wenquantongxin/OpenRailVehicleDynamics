import type { CSSProperties } from 'react';

import {
  elementKey,
  sameSubject,
  subjectElement,
  subjectKey,
  type CreationOffer,
  type ElementKindDescription,
  type ElementReference,
  type ModelTreeNode,
  type PropertySubject,
} from '../service/vehicle_modeling_service_protocol.ts';
import { BilingualTitleText } from './BilingualTitleText.tsx';
import { elementKindTitle } from './find_element_kind_title.ts';
import { elementRowsDirectlyUnder, hasCreateButton, matchingElementRowsDirectlyUnder, showsElementCount, showsKindTag, type RowsShownByFilter } from './model_tree_display_rules.ts';
import { WrappableElementName } from './WrappableElementName.tsx';

// One row of the model tree and, while it is open, the rows under it, drawn
// as the service describes them: an element's row shows the element's name and,
// in a list of types, its kind as a quiet tag; another row shows its title. A
// row that selects something selects it; a row that only groups opens and
// closes, except under a filter, which opens what it shows; a row not provided
// yet is greyed and takes no input. Indentation carries the level, and a
// titled row says how many element rows stand directly under it. A row's "+"
// creates what it offers, or attaches one of the bodies it lists.

/** What every row of one tree is drawn with. */
export interface ModelTreeRowSurroundings {
  kinds: readonly ElementKindDescription[];
  /** The kinds bodies are attached by. */
  attachingKinds: readonly string[];
  selection: PropertySubject | null;
  openRows: ReadonlySet<string>;
  /** What the filter shows; null while there is no filter. */
  filtered: RowsShownByFilter | null;
  /** The row whose "+" is in use. */
  creatingFromRow: string | null;
  onToggle: (key: string) => void;
  onSelect: (subject: PropertySubject) => void;
  onCreate: (rowKey: string, offers: CreationOffer[]) => void;
  onAttach: (rowKey: string, bodies: ElementReference[]) => void;
}

export function ModelTreeRow(props: { row: ModelTreeNode; parent: ModelTreeNode | null; depth: number; surroundings: ModelTreeRowSurroundings }) {
  const { row, depth, surroundings } = props;
  const { filtered } = surroundings;
  if (filtered !== null && !filtered.shown.has(row.key)) return null;
  const element = subjectElement(row.subject);
  const opens = row.provided && row.children.length > 0;
  const open = opens && (filtered !== null ? filtered.open.has(row.key) : surroundings.openRows.has(row.key));
  // Under a filter the filter opens the rows, so a row shows whether it is open but is not opened or closed by hand.
  const toggles = opens && filtered === null;
  const chevron = <span className={`chevron ${open ? 'open' : ''}`} aria-hidden="true" />;
  const selected = sameSubject(row.subject, surroundings.selection);
  const classes = ['tree-row', depth === 0 ? 'first-level' : '', element !== null ? 'element' : 'titled', row.provided ? '' : 'not-provided', selected ? 'selected' : '', surroundings.creatingFromRow === row.key ? 'creating' : ''];

  const label = (
    <>
      {element !== null ? (
        <span className="row-name">
          <WrappableElementName name={element.name} />
          {showsKindTag(props.parent, element, surroundings.attachingKinds) && (
            <span className="kind-tag">
              <BilingualTitleText title={elementKindTitle(surroundings.kinds, element.kind)} />
            </span>
          )}
        </span>
      ) : (
        row.title !== null && (
          <span className="row-title">
            <BilingualTitleText title={row.title} />
            {!row.provided && (
              <span className="not-provided-mark">
                Not provided yet <span className="zh">尚未提供</span>
              </span>
            )}
          </span>
        )
      )}
      {row.note !== null && (
        <span className="row-note">
          <BilingualTitleText title={row.note} />
        </span>
      )}
    </>
  );

  return (
    <div className="tree-branch">
      <div
        className={classes.filter((each) => each !== '').join(' ')}
        style={{ '--depth': depth } as CSSProperties}
        data-row={row.key}
        data-subject={row.subject === null ? undefined : subjectKey(row.subject)}
        data-element={element === null ? undefined : elementKey(element)}
      >
        {toggles ? (
          <button type="button" className="row-toggle" aria-expanded={open} aria-label={open ? 'Close' : 'Open'} onClick={() => surroundings.onToggle(row.key)}>
            {chevron}
          </button>
        ) : (
          <span className="row-toggle" aria-hidden="true">
            {opens && chevron}
          </span>
        )}
        {row.provided && row.subject !== null ? (
          <button type="button" className="row-label" aria-current={selected || undefined} onClick={() => surroundings.onSelect(row.subject!)}>
            {label}
          </button>
        ) : toggles ? (
          <button type="button" className="row-label" onClick={() => surroundings.onToggle(row.key)}>
            {label}
          </button>
        ) : (
          <span className="row-label">{label}</span>
        )}
        {showsElementCount(row) && (
          <span className="row-count">
            {filtered !== null ? `${matchingElementRowsDirectlyUnder(row, filtered.matching)} / ${elementRowsDirectlyUnder(row)}` : elementRowsDirectlyUnder(row)}
          </span>
        )}
        {hasCreateButton(row) && (
          <button
            type="button"
            className="create-button"
            aria-label="Create"
            title="Create · 新建"
            onClick={() => (row.creates.length > 0 ? surroundings.onCreate(row.key, row.creates) : surroundings.onAttach(row.key, row.attaches))}
          >
            <span aria-hidden="true">+</span>
          </button>
        )}
      </div>
      {open && row.children.map((child) => <ModelTreeRow key={child.key} row={child} parent={row} depth={depth + 1} surroundings={surroundings} />)}
    </div>
  );
}
