import { curveValueKind, type CurvePoint, type PropertyReading, type Rotation } from '../service/vehicle_modeling_service_protocol.ts';
import { identifierLabel } from './identifier_display_labels.ts';
import { WrappableElementName } from './WrappableElementName.tsx';

// A property value that is shown, not edited. Numbers keep their full
// precision; an absent value is a dash.

function RotationGrid({ rows }: { rows: Rotation }) {
  return (
    <span className="value rotation">
      {rows.flatMap((row, rowIndex) => row.map((entry, column) => <span key={`${rowIndex}.${column}`}>{String(entry)}</span>))}
    </span>
  );
}

/** A curve's rows as a table; the columns are the row fields the service names. */
function CurveTable({ rowFields, rows }: { rowFields: readonly string[]; rows: readonly CurvePoint[] }) {
  return (
    <table className="value curve">
      <thead>
        <tr>
          {rowFields.map((rowField) => (
            <th key={rowField}>{identifierLabel(rowField)}</th>
          ))}
        </tr>
      </thead>
      <tbody>
        {rows.map((row, index) => (
          <tr key={index}>
            {rowFields.map((rowField) => (
              <td key={rowField}>{String(row[rowField])}</td>
            ))}
          </tr>
        ))}
      </tbody>
    </table>
  );
}

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
      return <RotationGrid rows={property.value} />;
    case curveValueKind:
      return <CurveTable rowFields={property.rowFields} rows={property.value} />;
  }
}
