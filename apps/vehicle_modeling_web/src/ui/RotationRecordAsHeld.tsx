import { Fragment } from 'react';

import type { RotationReading } from '../service/vehicle_modeling_service_protocol.ts';

// What a rotation's record holds, shown under its angles, read-only and
// quiet: the matrix the service gives, its three rows one to a line, each
// entry with its full precision and never broken, a row that is too long for
// the panel going on under itself; and, where the record holds no rotation,
// one line saying so and that three angles replace it. Nothing is computed or
// repaired here.

export function RotationRecordAsHeld({ reading }: { reading: RotationReading }) {
  return (
    <div className="rotation-record">
      {reading.rollPitchYawRadians === null && (
        <p className="rotation-record-note">
          The record holds no rotation here; three angles replace what it holds. <span className="zh">记录在此处不是转动；输入三个角度即替换之。</span>
        </p>
      )}
      {reading.matrixRows !== null && (
        <>
          <span className="rotation-record-caption">
            Matrix as the record holds it <span className="zh">记录中的矩阵</span>
          </span>
          <span className="value matrix-rows">
            {reading.matrixRows.map((row, rowIndex) => (
              <span className="matrix-row" key={rowIndex}>
                {row.map((entry, column) => (
                  <Fragment key={column}>
                    {column > 0 && ' '}
                    <span className="matrix-entry">
                      {column === 0 && '['}
                      {String(entry)}
                      {column < 2 ? ',' : ']'}
                    </span>
                  </Fragment>
                ))}
              </span>
            ))}
          </span>
        </>
      )}
    </div>
  );
}
