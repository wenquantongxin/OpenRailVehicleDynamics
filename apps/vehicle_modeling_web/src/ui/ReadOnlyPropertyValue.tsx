import { curveValueKind, type PropertyReading } from '../service/vehicle_modeling_service_protocol.ts';
import { ReadOnlyCurveTable } from './ReadOnlyCurveTable.tsx';
import { rollPitchYawCaptions } from './roll_pitch_yaw_angle_captions.ts';
import { RollPitchYawAngleCaption } from './RollPitchYawAngleCaption.tsx';
import { RotationRecordAsHeld } from './RotationRecordAsHeld.tsx';
import { WrappableElementName } from './WrappableElementName.tsx';

// A property value that is shown, not edited. Numbers keep their full
// precision, a text is shown as it is, and an absent value is a dash. A
// rotation shows its angles, each after its caption, over what its record
// holds.

export function ReadOnlyPropertyValue({ property }: { property: PropertyReading }) {
  if (property.value === null) {
    return <span className="value absent">—</span>;
  }
  switch (property.valueKind) {
    case 'number':
      return <span className="value">{String(property.value)}</span>;
    case 'vector3':
      return (
        <span className="value vector">
          {property.value.map((entry, index) => (
            <span key={index}>{String(entry)}</span>
          ))}
        </span>
      );
    case 'element_name':
    case 'element_reference':
    case 'token':
      return (
        <span className="value">
          <WrappableElementName name={property.value} />
        </span>
      );
    case 'rotation': {
      const angles = property.value.rollPitchYawRadians;
      return (
        <>
          {angles !== null && (
            <span className="value angle-values">
              {rollPitchYawCaptions.map((caption, position) => (
                <span className="angle-field" key={caption.axis}>
                  <RollPitchYawAngleCaption caption={caption} />
                  <span>{String(angles[position])}</span>
                </span>
              ))}
            </span>
          )}
          <RotationRecordAsHeld reading={property.value} />
        </>
      );
    }
    case curveValueKind:
      return <ReadOnlyCurveTable rowFields={property.rowFields} rows={property.value} />;
    case 'text':
      return <span className="value text">{property.value}</span>;
  }
}
