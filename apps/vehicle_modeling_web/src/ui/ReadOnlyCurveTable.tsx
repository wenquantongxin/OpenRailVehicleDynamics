import type { CurvePoint } from '../service/vehicle_modeling_service_protocol.ts';
import { identifierLabel } from './identifier_display_labels.ts';

/** A curve shown, not edited: its rows as a table, whose columns are the row fields the service names. */
export function ReadOnlyCurveTable({ rowFields, rows }: { rowFields: readonly string[]; rows: readonly CurvePoint[] }) {
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
