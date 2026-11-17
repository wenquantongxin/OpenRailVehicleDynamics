import { curveValueKind, type ElementKindDescription, type ElementReference, type PropertyReading, type PropertySubject } from '../service/vehicle_modeling_service_protocol.ts';
import type { EditRefusal, SetProperty } from '../session/vehicle_editing_session.ts';
import { BooleanPropertyEditor } from './BooleanPropertyEditor.tsx';
import { CurvePropertyEditor } from './CurvePropertyEditor.tsx';
import { ElementNamePropertyEditor } from './ElementNamePropertyEditor.tsx';
import { ElementReferencePropertyEditor } from './ElementReferencePropertyEditor.tsx';
import { NumberAndVectorPropertyEditor } from './NumberAndVectorPropertyEditor.tsx';
import { ReadOnlyPropertyValue } from './ReadOnlyPropertyValue.tsx';
import { TokenPropertyEditor } from './TokenPropertyEditor.tsx';

/** One property of the selected subject, as its row and its editor are given it. */
export interface SubjectPropertyRowProps {
  subject: PropertySubject;
  property: PropertyReading;
  /** The outline's elements, which a reference may name. */
  elements: readonly ElementReference[];
  /** The kinds' descriptions, whose titles name the kinds a reference may name. */
  kinds: readonly ElementKindDescription[];
  refusal: EditRefusal | null;
  /** The field takes no input: the element's rename is on its way. */
  frozen: boolean;
  onCommit: SetProperty;
  onDismissRefusal: () => void;
}

/**
 * The property's value in the editor of its value kind when it is editable;
 * otherwise only shown, as rotations and texts always are. A number the
 * subject holds no value for yet is entered in an empty field.
 */
export function SubjectPropertyValueEditor(props: SubjectPropertyRowProps) {
  const { property } = props;
  const committing = { subject: props.subject, refusal: props.refusal, onCommit: props.onCommit };
  const typing = { ...committing, propertyName: property.name, onDismissRefusal: props.onDismissRefusal };
  const { frozen } = props;
  if (!property.editable) return <ReadOnlyPropertyValue property={property} />;
  switch (property.valueKind) {
    case 'number':
      return <NumberAndVectorPropertyEditor {...typing} frozen={frozen} current={[property.value]} />;
    case 'vector3':
      return <NumberAndVectorPropertyEditor {...typing} frozen={frozen} current={property.value ?? [null, null, null]} />;
    case 'element_name':
      return <ElementNamePropertyEditor {...typing} current={property.value ?? ''} />;
    case 'element_reference':
      return <ElementReferencePropertyEditor {...committing} frozen={frozen} description={property} value={property.value} elements={props.elements} kinds={props.kinds} />;
    case 'token':
      return <TokenPropertyEditor {...committing} frozen={frozen} propertyName={property.name} choices={property.choices} value={property.value} />;
    case 'boolean':
      return <BooleanPropertyEditor {...committing} frozen={frozen} propertyName={property.name} value={property.value} />;
    case curveValueKind:
      return <CurvePropertyEditor {...typing} frozen={frozen} rowFields={property.rowFields} current={property.value ?? []} />;
    case 'rotation':
    case 'text':
      return <ReadOnlyPropertyValue property={property} />;
  }
}
