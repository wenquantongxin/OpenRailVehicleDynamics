import { isRotationByAngles, rotationByAngles, type PropertySubject, type RotationReading, type Vector3 } from '../service/vehicle_modeling_service_protocol.ts';
import type { EditRefusal, SetProperty } from '../session/vehicle_editing_session.ts';
import { readNumberEntry } from './decimal_number_text_reading.ts';
import { FieldRefusalMessage } from './FieldRefusalMessage.tsx';
import { useTypedFieldCommit } from './property_field_commit_hooks.ts';
import { RollPitchYawAngleInputs } from './RollPitchYawAngleInputs.tsx';
import { RotationRecordAsHeld } from './RotationRecordAsHeld.tsx';
import { takesTyping } from './typed_property_field_commit_rules.ts';
import { UnreadableNumberMessage } from './UnreadableNumberMessage.tsx';

// An editable rotation of the selected subject: its three angles in radians,
// typed and committed as a vector's three numbers are, and sent together; the
// angles shown are the ones the service reads back. Under them what the
// record holds, as the service gives it. Where the record holds no rotation
// the angle fields are empty, and three angles replace what it holds.

export function RotationAnglesPropertyEditor(props: {
  subject: PropertySubject;
  propertyName: string;
  /** Null where the subject holds no value yet. */
  current: RotationReading | null;
  refusal: EditRefusal | null;
  /** Takes no input: the element's rename is on its way. */
  frozen: boolean;
  onCommit: SetProperty;
  onDismissRefusal: () => void;
}) {
  const { subject, propertyName, current, refusal } = props;
  const angles = current?.rollPitchYawRadians ?? [null, null, null];
  const shown = angles.map((angle) => (angle === null ? '' : String(angle)));
  const { field, type, commit, revert } = useTypedFieldCommit<readonly string[], number[]>({
    // A field shown again after its edit was refused starts from the refused angles.
    refusedEntry: refusal !== null && isRotationByAngles(refusal.value) ? refusal.value.roll_pitch_yaw_radians.map(String) : null,
    read: (texts) => readNumberEntry(texts, angles),
    send: (values) => props.onCommit(subject, propertyName, rotationByAngles(values as Vector3)),
    onRevert: () => {
      if (refusal !== null) props.onDismissRefusal();
    },
  });
  const texts = field.entry ?? shown;
  return (
    <>
      <div
        className="number-fields angle-fields"
        onBlur={(event) => {
          // Moving between the three inputs is not leaving the field.
          if (!event.currentTarget.contains(event.relatedTarget as Node | null)) void commit();
        }}
      >
        <RollPitchYawAngleInputs
          texts={texts}
          changed={texts.map((text, position) => field.entry !== null && text !== shown[position])}
          unreadableAt={field.unreadableAt}
          refused={refusal !== null}
          readOnly={!takesTyping(field) || props.frozen}
          onText={(position, text) => type((entry) => (entry ?? shown).map((each, index) => (index === position ? text : each)))}
          onKeyDown={(event) => {
            if (event.key === 'Enter') void commit();
            else if (event.key === 'Escape') revert();
          }}
        />
        {field.unreadableAt !== null && <UnreadableNumberMessage />}
        {refusal !== null && <FieldRefusalMessage message={refusal.message} />}
      </div>
      {current !== null && <RotationRecordAsHeld reading={current} />}
    </>
  );
}
