import type { CurvePoint, PropertyReading, Rotation } from '../service/protocol.ts';
import { ElementName } from './ElementName.tsx';
import { identifierLabel } from './identifier_label.ts';

// A property value that is shown, not edited. Numbers keep their full
// precision; an absent value is a dash.

function RotationGrid({ rows }: { rows: Rotation }) {
  return (
    <span className="value rotation">
      {rows.flatMap((row, rowIndex) => row.map((entry, column) => <span key={`${rowIndex}.${column}`}>{String(entry)}</span>))}
    </span>
  );
}

/** A curve's points as a table; the columns are the point fields the service names. */
function CurveTable({ points }: { points: CurvePoint[] }) {
  const columns = Object.keys(points[0] ?? {});
  return (
    <table className="value curve">
      <thead>
        <tr>
          {columns.map((column) => (
            <th key={column}>{identifierLabel(column)}</th>
          ))}
        </tr>
      </thead>
      <tbody>
        {points.map((point, index) => (
          <tr key={index}>
            {columns.map((column) => (
              <td key={column}>{String(point[column])}</td>
            ))}
          </tr>
        ))}
      </tbody>
    </table>
  );
}

export function PropertyValueView({ property }: { property: PropertyReading }) {
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
    case 'text':
    case 'wheel_side':
      return (
        <span className="value">
          <ElementName name={property.value} />
        </span>
      );
    case 'rotation':
      return <RotationGrid rows={property.value} />;
    case 'saturated_piecewise_linear_damper_curve':
      return <CurveTable points={property.value} />;
  }
}
