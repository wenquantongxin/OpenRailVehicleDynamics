export const elementLabelWidthPixels = 20;
const labelGapPixels = 10;

/** Thin the text, not the element ticks. Positions are fractions of the time bar. */
export function layoutElementLabels<T extends { at: number }>(points: readonly T[], widthPixels: number): { point: T; leftPixels: number }[] {
  if (widthPixels < elementLabelWidthPixels) return [];
  const labels: { point: T; leftPixels: number }[] = [];
  let nextLeft = 0;
  for (const point of [...points].sort((a, b) => a.at - b.at)) {
    if (!Number.isFinite(point.at) || point.at < 0 || point.at > 1) continue;
    const leftPixels = Math.max(0, Math.min(widthPixels - elementLabelWidthPixels, point.at * widthPixels - elementLabelWidthPixels / 2));
    if (leftPixels < nextLeft) continue;
    labels.push({ point, leftPixels });
    nextLeft = leftPixels + elementLabelWidthPixels + labelGapPixels;
  }
  return labels;
}
