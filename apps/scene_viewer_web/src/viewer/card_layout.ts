import type { SafeArea } from '../scene/label_layer.ts';
import type { ViewportArea } from '../scene/camera_framing.ts';

// The margins of the stage covered by the card columns and the playback card,
// measured from the elements themselves so the CSS stays the only place the
// layout is written down. Each margin is widened by the card's own inset from
// the stage edge, so the free picture keeps the same clearance from a card
// that the card keeps from the edge. An absent card contributes nothing.

/** The four edges of a DOMRect, so the measurement is testable without a DOM. */
export interface EdgeRect {
  left: number;
  right: number;
  top: number;
  bottom: number;
}

export function measureCardLayout(stage: EdgeRect, leftColumn: EdgeRect | null, rightColumn: EdgeRect | null, playbackCard: EdgeRect | null): SafeArea {
  const left = leftColumn === null ? 0 : Math.max(0, leftColumn.right - stage.left + (leftColumn.left - stage.left));
  const right = rightColumn === null ? 0 : Math.max(0, stage.right - rightColumn.left + (stage.right - rightColumn.right));
  const columnTops = [leftColumn, rightColumn].filter((column): column is EdgeRect => column !== null).map((column) => column.top - stage.top);
  const top = columnTops.length === 0 ? 0 : Math.max(0, Math.min(...columnTops));
  const bottom = playbackCard === null ? 0 : Math.max(0, stage.bottom - playbackCard.top + (stage.bottom - playbackCard.bottom));
  return { left, right, top, bottom };
}

/** Keep every positive measured opening, however narrow. Only an empty opening has no usable fit area. */
export function viewportAreaFromLayout(width: number, height: number, layout: SafeArea): ViewportArea {
  let x0 = layout.left;
  let x1 = width - layout.right;
  let y0 = layout.top;
  let y1 = height - layout.bottom;
  if (x1 <= x0) {
    x0 = 0;
    x1 = width;
  }
  if (y1 <= y0) {
    y0 = 0;
    y1 = height;
  }
  return { width, height, free: { x0, y0, x1, y1 } };
}
