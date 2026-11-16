import { curveValueKind, type PropertyDescription } from '../service/vehicle_modeling_service_protocol.ts';
import { ElementPropertyValueEditor, type ElementPropertyRowProps } from './ElementPropertyValueEditor.tsx';
import { identifierLabel } from './identifier_display_labels.ts';

// One property of the selected element: its name, its value kind, and its
// value, in the editor of its value kind when it is editable. Rotations are
// only shown.

/** How a property's value kind reads in its head. */
export function valueKindLabel(property: PropertyDescription): string {
  return property.valueKind === curveValueKind ? 'curve' : identifierLabel(property.valueKind);
}

export function ElementPropertyRow(props: ElementPropertyRowProps) {
  const { property } = props;
  return (
    <div className={`property ${property.editable ? 'editable' : 'read-only'}`} data-property={property.name}>
      <div className="property-head">
        <span className="property-name">{identifierLabel(property.name)}</span>
        <span className="property-kind">{valueKindLabel(property)}</span>
      </div>
      <ElementPropertyValueEditor {...props} />
    </div>
  );
}
