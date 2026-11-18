// The three angles of a rotation as the interface names them: each by the
// axis it turns about and its unit. The service gives and takes them in this
// order, roll, pitch and yaw, turned about the fixed axes in that order.

export interface AngleCaption {
  axis: 'x' | 'y' | 'z';
  en: string;
  zh: string;
}

export const rollPitchYawCaptions: readonly AngleCaption[] = [
  { axis: 'x', en: 'roll', zh: '滚转' },
  { axis: 'y', en: 'pitch', zh: '俯仰' },
  { axis: 'z', en: 'yaw', zh: '偏航' },
];

/** What an angle's input is called for assistive technology: its name, its axis and its unit. */
export function angleInputLabel(caption: AngleCaption): string {
  return `${caption.en} about ${caption.axis}, in radians`;
}
