import { useEffect, useRef, type KeyboardEvent } from 'react';

import type { CurveEntry } from './editable_curve_table_rules.ts';
import { identifierLabel } from './identifier_display_labels.ts';

// A curve as a small table: a column for each row field the service names, a
// row per point, a button to remove each row and one to add a row at the end,
// also the first row of an empty curve. A row added takes the focus.

export function EditableCurveTable(props: {
  propertyName: string;
  rowFields: readonly string[];
  entry: CurveEntry;
  /** Whether a cell's text differs from the curve shown. */
  changed: (row: number, column: number) => boolean;
  /** The cell whose text is no number, counted row by row. */
  unreadableAt: number | null;
  refused: boolean;
  readOnly: boolean;
  onCellText: (row: number, column: number, text: string) => void;
  onAddRow: () => void;
  onRemoveRow: (row: number) => void;
  onKeyDown?: (event: KeyboardEvent<HTMLInputElement>) => void;
}) {
  const { rowFields, entry } = props;
  const table = useRef<HTMLTableElement>(null);
  const addButton = useRef<HTMLButtonElement>(null);
  const rowsBefore = useRef(entry.length);
  useEffect(() => {
    if (entry.length > rowsBefore.current) table.current?.querySelector<HTMLInputElement>('tbody tr:last-child input')?.focus();
    rowsBefore.current = entry.length;
  }, [entry.length]);
  const label = identifierLabel(props.propertyName);

  return (
    <div className="curve-rows">
      {entry.length === 0 ? (
        <p className="curve-empty">
          No rows yet <span className="zh">尚无数据行</span>
        </p>
      ) : (
        <table className="curve-table" ref={table}>
          <thead>
            <tr>
              {rowFields.map((rowField) => (
                <th key={rowField}>{identifierLabel(rowField)}</th>
              ))}
              <th aria-hidden="true" />
            </tr>
          </thead>
          <tbody>
            {entry.map((texts, row) => (
              <tr key={row}>
                {texts.map((text, column) => (
                  <td key={column}>
                    <input
                      type="text"
                      inputMode="decimal"
                      spellCheck={false}
                      autoComplete="off"
                      aria-label={`${label} · ${identifierLabel(rowFields[column]!)} [${row}]`}
                      aria-invalid={props.unreadableAt === row * rowFields.length + column || props.refused}
                      className={props.changed(row, column) ? 'changed' : ''}
                      readOnly={props.readOnly}
                      value={text}
                      onChange={(event) => props.onCellText(row, column, event.target.value)}
                      onKeyDown={props.onKeyDown}
                    />
                  </td>
                ))}
                <td className="row-action">
                  <button
                    type="button"
                    className="row-remove"
                    title="Remove row · 删除行"
                    aria-label={`Remove row ${row + 1}`}
                    disabled={props.readOnly}
                    onClick={() => {
                      props.onRemoveRow(row);
                      // The focus stays in the table, so that leaving it is what commits.
                      addButton.current?.focus();
                    }}
                  >
                    ×
                  </button>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      )}
      <button type="button" className="chip row-add" ref={addButton} disabled={props.readOnly} onClick={props.onAddRow}>
        <span className="en">+ Add row</span>
        <span className="zh">添加行</span>
      </button>
    </div>
  );
}
