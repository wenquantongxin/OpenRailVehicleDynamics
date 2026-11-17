import type { AttachmentDescription, ElementKindDescription, ElementReference } from '../service/vehicle_modeling_service_protocol.ts';
import { formProperties, type FormEntries, type FormEntry } from './element_creation_form_rules.ts';
import { ElementCreationPropertyInput } from './ElementCreationPropertyInput.tsx';
import { FieldRefusalMessage } from './FieldRefusalMessage.tsx';
import { identifierLabel } from './identifier_display_labels.ts';
import { valueKindLabel } from './SubjectPropertyRow.tsx';
import { UnreadableNumberMessage } from './UnreadableNumberMessage.tsx';

// The fields of the joint a body is to hang on, as its kind asks for them: its
// name, its ends and its other properties, empty where nothing is carried
// over. An end that may name only some frames lists only those; any other
// reference lists every element of the outline it may name.

export function JointFormFields(props: {
  /** The joint kind as its fields ask for it. */
  jointKind: ElementKindDescription;
  description: AttachmentDescription;
  entries: FormEntries;
  /** What the end on the body may name; null where any element it may name will do. */
  bodyEndChoices: readonly ElementReference[] | null;
  /** What the other end may name; null where any element it may name will do. */
  otherEndChoices: readonly ElementReference[] | null;
  elements: readonly ElementReference[];
  kinds: readonly ElementKindDescription[];
  disabled: boolean;
  unreadable: { property: string; at: number } | null;
  /** A refusal shown at the field it names. */
  refusal: { property: string; message: string } | null;
  onEntry: (property: string, entry: FormEntry) => void;
}) {
  const choicesFor = (property: string): readonly ElementReference[] =>
    (property === props.description.bodyEndProperty ? props.bodyEndChoices : property === props.description.otherEndProperty ? props.otherEndChoices : null) ?? props.elements;
  return (
    <div className="joint-fields">
      {formProperties(props.jointKind).map((property) => (
        <div className="property" data-property={property.name} key={property.name}>
          <div className="property-head">
            <span className="property-name">{identifierLabel(property.name)}</span>
            <span className="property-kind">{valueKindLabel(property)}</span>
          </div>
          <ElementCreationPropertyInput
            property={property}
            entry={props.entries[property.name]!}
            elements={choicesFor(property.name)}
            kinds={props.kinds}
            disabled={props.disabled}
            unreadableAt={props.unreadable?.property === property.name ? props.unreadable.at : null}
            refused={props.refusal?.property === property.name}
            onEntry={(entry) => props.onEntry(property.name, entry)}
          />
          {props.unreadable?.property === property.name && <UnreadableNumberMessage />}
          {props.refusal?.property === property.name && <FieldRefusalMessage message={props.refusal.message} />}
        </div>
      ))}
    </div>
  );
}
