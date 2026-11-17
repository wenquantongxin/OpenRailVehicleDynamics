import { curveValueKind, type PropertyReading } from '../service/vehicle_modeling_service_protocol.ts';
import { ReadOnlyCurveTable } from './ReadOnlyCurveTable.tsx';
import { ReadOnlyRotationGrid } from './ReadOnlyRotationGrid.tsx';
import { WrappableElementName } from './WrappableElementName.tsx';

// A property value that is shown, not edited. Numbers keep their full
// precision, a text is shown as it is, and an absent value is a dash.

export function ReadOnlyPropertyValue({ property }: { property: PropertyReading }) {
  if (property.value === null) {
    return <span className="value absent">—</span>;
  }
  switch (property.valueKind) {
    case 'number':
    case 'boolean':
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
    case 'rotation':
      return <ReadOnlyRotationGrid rows={property.value} />;
    case curveValueKind:
      return <ReadOnlyCurveTable rowFields={property.rowFields} rows={property.value} />;
    case 'text':
      return <span className="value text">{property.value}</span>;
  }
}
