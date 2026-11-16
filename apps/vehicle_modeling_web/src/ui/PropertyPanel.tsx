import { useEffect, useRef, useState } from 'react';

import { elementKey, sameElement, type ElementReference, type PropertyReading, type Vector3 } from '../service/protocol.ts';
import type { EditRefusal, EditValue } from '../session/editing_session.ts';
import { ElementName } from './ElementName.tsx';
import { identifierLabel } from './identifier_label.ts';
import { afterAnswer, afterCommit, afterTyping, restingField, takesTyping, type NumberFieldState } from './number_field.ts';
import { PanelHead } from './PanelHead.tsx';
import { PropertyValueView } from './PropertyValueView.tsx';

// The selected element's properties, in the service's order. An editable
// number or vector is entered in fields; everything else is shown. A field
// commits on Enter or when focus leaves it, Escape reverts it, and nothing is
// sent when its value did not change. A refused edit keeps the text entered
// and shows the service's reason under the field.

/** Asks the panel to focus one property's field once it shows that element. */
export interface FocusRequest {
  element: ElementReference;
  property: string;
  serial: number;
}

type Commit = (element: ElementReference, property: string, value: EditValue) => Promise<boolean>;

/**
 * What an editable number or vector shows in its fields: one number or three;
 * null for a property that is only shown. A value the element does not hold is
 * never editable, and is shown as a dash.
 */
function fieldValues(property: PropertyReading): number[] | null {
  if (!property.editable || property.value === null) return null;
  if (property.valueKind === 'number') return [property.value];
  if (property.valueKind === 'vector3') return property.value;
  return null;
}

function NumberFields(props: {
  element: ElementReference;
  property: PropertyReading;
  current: number[];
  refusal: EditRefusal | null;
  onCommit: Commit;
  onDismissRefusal: () => void;
}) {
  const { element, property, current, refusal } = props;
  // A field shown again after its edit was refused starts from the refused value.
  const [field, showField] = useState<NumberFieldState>(() => (refusal === null ? restingField : { ...restingField, draft: [refusal.value].flat().map(String) }));
  // Every event reads the field as the last one left it, rendered yet or not.
  const latest = useRef(field);
  const setField = (next: NumberFieldState): void => {
    latest.current = next;
    showField(next);
  };
  const refusalMessage = useRef<HTMLParagraphElement>(null);
  const texts = field.draft ?? current.map(String);

  useEffect(() => {
    refusalMessage.current?.scrollIntoView({ block: 'nearest' });
  }, [refusal]);

  const revert = (): void => {
    if (!takesTyping(latest.current)) return;
    setField(restingField);
    if (refusal !== null) props.onDismissRefusal();
  };

  const commit = async (): Promise<void> => {
    const step = afterCommit(latest.current, current);
    if (step.reverted) {
      revert();
      return;
    }
    setField(step.state);
    if (step.send === null) return;
    const value = current.length === 1 ? step.send[0]! : (step.send as Vector3);
    const accepted = await props.onCommit(element, property.name, value);
    setField(afterAnswer(latest.current, accepted));
  };

  return (
    <div
      className={`number-fields count-${texts.length}`}
      onBlur={(event) => {
        // Moving between a vector's own inputs is not leaving the field.
        if (!event.currentTarget.contains(event.relatedTarget as Node | null)) void commit();
      }}
    >
      {texts.map((text, position) => (
        <input
          key={position}
          type="text"
          inputMode="decimal"
          spellCheck={false}
          autoComplete="off"
          aria-label={texts.length === 1 ? identifierLabel(property.name) : `${identifierLabel(property.name)} [${position}]`}
          aria-invalid={field.notANumberAt === position || refusal !== null}
          className={field.draft !== null && text !== String(current[position]) ? 'changed' : ''}
          readOnly={!takesTyping(field)}
          value={text}
          onChange={(event) => {
            const shown = latest.current.draft ?? current.map(String);
            setField(afterTyping(latest.current, shown.map((entry, index) => (index === position ? event.target.value : entry))));
          }}
          onKeyDown={(event) => {
            if (event.key === 'Enter') void commit();
            else if (event.key === 'Escape') revert();
          }}
        />
      ))}
      {field.notANumberAt !== null && (
        <p className="field-message">
          Enter a decimal number, e.g. 2.5e5 <span className="zh">请输入十进制数</span>
        </p>
      )}
      {refusal !== null && (
        <p className="field-message refused" role="alert" ref={refusalMessage}>
          {refusal.message}
        </p>
      )}
    </div>
  );
}

function PropertyRow(props: {
  element: ElementReference;
  property: PropertyReading;
  refusal: EditRefusal | null;
  onCommit: Commit;
  onDismissRefusal: () => void;
}) {
  const { property } = props;
  const current = fieldValues(property);
  return (
    <div className={`property ${property.editable ? 'editable' : 'read-only'}`} data-property={property.name}>
      <div className="property-head">
        <span className="property-name">{identifierLabel(property.name)}</span>
        <span className="property-kind">{property.valueKind}</span>
      </div>
      {current === null ? <PropertyValueView property={property} /> : <NumberFields {...props} current={current} />}
    </div>
  );
}

export function PropertyPanel(props: {
  selection: ElementReference | null;
  properties: PropertyReading[];
  editRefusal: EditRefusal | null;
  focusRequest: FocusRequest | null;
  onCommit: Commit;
  onDismissRefusal: () => void;
}) {
  const { selection, editRefusal, focusRequest } = props;
  const body = useRef<HTMLDivElement>(null);
  const focusedSerial = useRef(0);

  useEffect(() => {
    if (focusRequest === null || focusRequest.serial === focusedSerial.current || !sameElement(focusRequest.element, selection)) return;
    const input = body.current?.querySelector<HTMLInputElement>(`[data-property="${CSS.escape(focusRequest.property)}"] input`);
    if (input === null || input === undefined) return;
    focusedSerial.current = focusRequest.serial;
    input.focus();
    input.select();
  }, [focusRequest, selection, props.properties]);

  return (
    <aside className="panel properties-panel">
      <PanelHead en="Properties" zh="属性" detail={selection === null ? undefined : `${props.properties.length}`} />
      {selection === null ? (
        <p className="panel-empty">
          Select an element in the tree or the 3D view. <span className="zh">在组件树或三维视图中选择元素。</span>
        </p>
      ) : (
        <>
          <div className="selection-head">
            <span className="selection-kind">{identifierLabel(selection.kind)}</span>
            <span className="selection-name">
              <ElementName name={selection.name} />
            </span>
          </div>
          <div className="property-list" ref={body}>
            {props.properties.map((property) => (
              <PropertyRow
                key={`${elementKey(selection)}/${property.name}`}
                element={selection}
                property={property}
                refusal={editRefusal !== null && sameElement(editRefusal.element, selection) && editRefusal.property === property.name ? editRefusal : null}
                onCommit={props.onCommit}
                onDismissRefusal={props.onDismissRefusal}
              />
            ))}
          </div>
        </>
      )}
    </aside>
  );
}
