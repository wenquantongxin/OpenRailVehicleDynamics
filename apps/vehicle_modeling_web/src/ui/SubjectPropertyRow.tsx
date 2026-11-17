import { curveValueKind, type PropertyDescription } from '../service/vehicle_modeling_service_protocol.ts';
import { identifierLabel } from './identifier_display_labels.ts';
import { SubjectPropertyValueEditor, type SubjectPropertyRowProps } from './SubjectPropertyValueEditor.tsx';

// One property of the selected subject: its name, its value kind, and its
// value, in the editor of its value kind when it is editable. A property that
// is not editable is only shown, and says so.

/** How a property's value kind reads in its head. */
export function valueKindLabel(property: PropertyDescription): string {
  return property.valueKind === curveValueKind ? 'curve' : identifierLabel(property.valueKind);
}

export function SubjectPropertyRow(props: SubjectPropertyRowProps) {
  const { property } = props;
  return (
    <div className={`property ${property.editable ? 'editable' : 'read-only'}`} data-property={property.name}>
      <div className="property-head">
        <span className="property-name">{identifierLabel(property.name)}</span>
        <span className="property-kind">
          {valueKindLabel(property)} ·{' '}
          {property.editable ? (
            <>
              editable <span className="zh">可编辑</span>
            </>
          ) : (
            <>
              read-only <span className="zh">只读</span>
            </>
          )}
        </span>
      </div>
      <SubjectPropertyValueEditor {...props} />
    </div>
  );
}
