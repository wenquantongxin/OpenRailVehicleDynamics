import type { CurvePoint, ElementReference } from '../service/vehicle_modeling_service_protocol.ts';
import type { EditRefusal, SetProperty } from '../session/vehicle_editing_session.ts';
import { curveEntryOf, readCurveEntry, withCellText, withRowAdded, withRowRemoved, type CurveEntry } from './editable_curve_table_rules.ts';
import { EditableCurveTable } from './EditableCurveTable.tsx';
import { FieldRefusalMessage } from './FieldRefusalMessage.tsx';
import { useTypedFieldCommit } from './property_field_commit_hooks.ts';
import { takesTyping } from './typed_property_field_commit_rules.ts';
import { UnreadableNumberMessage } from './UnreadableNumberMessage.tsx';

// An editable curve of the selected element. Rows are changed, added and
// removed in the table, and the whole curve is committed as one edit on
// Enter or when focus leaves the table; Escape reverts it.

export function CurvePropertyEditor(props: {
  element: ElementReference;
  propertyName: string;
  rowFields: readonly string[];
  current: readonly CurvePoint[];
  refusal: EditRefusal | null;
  /** Takes no input: the element's rename is on its way. */
  frozen: boolean;
  onCommit: SetProperty;
  onDismissRefusal: () => void;
}) {
  const { element, propertyName, rowFields, current, refusal } = props;
  const shown = curveEntryOf(current, rowFields);
  const { field, type, commit, revert } = useTypedFieldCommit<CurveEntry, CurvePoint[]>({
    refusedEntry: refusal === null ? null : curveEntryOf(refusal.value as readonly CurvePoint[], rowFields),
    read: (entry) => readCurveEntry(entry, rowFields, current),
    send: (curve) => props.onCommit(element, propertyName, curve),
    onRevert: () => {
      if (refusal !== null) props.onDismissRefusal();
    },
  });
  const entry = field.entry ?? shown;
  return (
    <div
      className="curve-field"
      onBlur={(event) => {
        if (!event.currentTarget.contains(event.relatedTarget as Node | null)) void commit();
      }}
    >
      <EditableCurveTable
        propertyName={propertyName}
        rowFields={rowFields}
        entry={entry}
        changed={(row, column) => field.entry !== null && entry[row]?.[column] !== shown[row]?.[column]}
        unreadableAt={field.unreadableAt}
        refused={refusal !== null}
        readOnly={!takesTyping(field) || props.frozen}
        onCellText={(row, column, text) => type((latest) => withCellText(latest ?? shown, row, column, text))}
        onAddRow={() => type((latest) => withRowAdded(latest ?? shown, rowFields))}
        onRemoveRow={(row) => type((latest) => withRowRemoved(latest ?? shown, row))}
        onKeyDown={(event) => {
          if (event.key === 'Enter') void commit();
          else if (event.key === 'Escape') revert();
        }}
      />
      {field.entry !== null && takesTyping(field) && (
        <p className="field-hint">
          Enter, or leaving the table, commits the curve; Escape reverts it. <span className="zh">回车或离开表格即提交整条曲线，Esc 撤回。</span>
        </p>
      )}
      {field.unreadableAt !== null && <UnreadableNumberMessage />}
      {refusal !== null && <FieldRefusalMessage message={refusal.message} />}
    </div>
  );
}
