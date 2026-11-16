import type { Rotation } from '../service/vehicle_modeling_service_protocol.ts';

/** A rotation shown, not edited: its three rows of a rotation matrix, each entry with its full precision. */
export function ReadOnlyRotationGrid({ rows }: { rows: Rotation }) {
  return (
    <span className="value rotation">
      {rows.flatMap((row, rowIndex) => row.map((entry, column) => <span key={`${rowIndex}.${column}`}>{String(entry)}</span>))}
    </span>
  );
}
