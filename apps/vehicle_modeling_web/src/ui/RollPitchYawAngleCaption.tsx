import type { AngleCaption } from './roll_pitch_yaw_angle_captions.ts';

/** An angle's caption: its axis, in the colour that axis has in the 3D view, its name and its unit. */
export function RollPitchYawAngleCaption({ caption }: { caption: AngleCaption }) {
  return (
    <span className="angle-caption">
      <b className={`axis-letter axis-${caption.axis}`}>{caption.axis}</b>
      <span className="en">{caption.en}</span>
      <span className="zh">{caption.zh}</span>
      <span className="unit">rad</span>
    </span>
  );
}
