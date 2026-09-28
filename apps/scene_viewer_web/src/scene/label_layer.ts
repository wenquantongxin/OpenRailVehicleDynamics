import * as THREE from 'three';

import type { SceneLabel } from './scene_builder.ts';

// Part names with leader lines. Each label is placed near its own anchor, in
// the closest spot that stays inside the free area, keeps clear of the
// bogies' on-screen outlines, overlaps no other label, covers no anchor and
// crosses no leader line. A label keeps its spot while that spot stays valid,
// so labels do not jump while the record plays; a label with no valid spot is
// hidden rather than drawn over another.

export interface SafeArea {
  left: number;
  right: number;
  top: number;
  bottom: number;
}

interface Spot {
  side: -1 | 1;
  dx: number;
  dy: number;
}

interface Item {
  label: SceneLabel;
  box: HTMLDivElement;
  path: SVGPathElement;
  dot: SVGRectElement;
  width: number;
  height: number;
  spot: Spot | null;
}

export interface Rect {
  x0: number;
  y0: number;
  x1: number;
  y1: number;
}

interface Segment {
  ax: number;
  ay: number;
  bx: number;
  by: number;
}

const svgNamespace = 'http://www.w3.org/2000/svg';
const distances = [56, 100, 150, 210, 280, 360];
const heights = [0, -44, 44, -88, 88, -132, 132, -176, 176, -220, 220, -264, 264];
const margin = 6;
/** Parts that subtend less than this angle (radians, roughly) are too small to name. */
const smallestAngle = 0.01;

function overlaps(a: Rect, b: Rect, pad: number): boolean {
  return a.x0 < b.x1 + pad && b.x0 < a.x1 + pad && a.y0 < b.y1 + pad && b.y0 < a.y1 + pad;
}

function contains(rect: Rect, x: number, y: number, pad: number): boolean {
  return x > rect.x0 - pad && x < rect.x1 + pad && y > rect.y0 - pad && y < rect.y1 + pad;
}

/** Whether a segment passes through a rectangle (Liang–Barsky clipping). */
function crosses(segment: Segment, rect: Rect): boolean {
  let t0 = 0;
  let t1 = 1;
  const dx = segment.bx - segment.ax;
  const dy = segment.by - segment.ay;
  const edges: [number, number][] = [
    [-dx, segment.ax - rect.x0],
    [dx, rect.x1 - segment.ax],
    [-dy, segment.ay - rect.y0],
    [dy, rect.y1 - segment.ay],
  ];
  for (const [p, q] of edges) {
    if (p === 0) {
      if (q < 0) {
        return false;
      }
    } else {
      const r = q / p;
      if (p < 0) {
        t0 = Math.max(t0, r);
      } else {
        t1 = Math.min(t1, r);
      }
      if (t0 > t1) {
        return false;
      }
    }
  }
  return true;
}

function segmentsIntersect(a: Segment, b: Segment): boolean {
  const cross = (px: number, py: number, qx: number, qy: number, rx: number, ry: number): number =>
    (qx - px) * (ry - py) - (qy - py) * (rx - px);
  const d1 = cross(b.ax, b.ay, b.bx, b.by, a.ax, a.ay);
  const d2 = cross(b.ax, b.ay, b.bx, b.by, a.bx, a.by);
  const d3 = cross(a.ax, a.ay, a.bx, a.by, b.ax, b.ay);
  const d4 = cross(a.ax, a.ay, a.bx, a.by, b.bx, b.by);
  return d1 * d2 < 0 && d3 * d4 < 0;
}

export class LabelLayer {
  private readonly element: HTMLDivElement;
  private readonly svg: SVGSVGElement;
  private readonly items: Item[];
  private visible = false;
  private readonly world = new THREE.Vector3();

  constructor(container: HTMLElement, labels: SceneLabel[]) {
    this.element = document.createElement('div');
    this.element.className = 'label-layer';
    this.svg = document.createElementNS(svgNamespace, 'svg');
    this.svg.setAttribute('class', 'label-lines');
    this.element.appendChild(this.svg);
    // Larger parts are placed first and so get the closest spots.
    const ordered = [...labels].sort((a, b) => b.sizeMeters - a.sizeMeters);
    this.items = ordered.map((label) => {
      const box = document.createElement('div');
      box.className = 'scene-label';
      const en = document.createElement('span');
      en.className = 'en';
      en.textContent = label.en;
      const zh = document.createElement('span');
      zh.className = 'zh';
      zh.textContent = label.zh;
      box.append(en, zh);
      this.element.appendChild(box);
      const path = document.createElementNS(svgNamespace, 'path');
      const dot = document.createElementNS(svgNamespace, 'rect');
      dot.setAttribute('width', '7');
      dot.setAttribute('height', '7');
      this.svg.append(path, dot);
      return { label, box, path, dot, width: 0, height: 0, spot: null };
    });
    container.appendChild(this.element);
    this.setVisible(false);
  }

  setVisible(visible: boolean): void {
    this.visible = visible;
    this.element.style.display = visible ? 'block' : 'none';
    if (visible) {
      for (const item of this.items) {
        item.width = item.box.offsetWidth;
        item.height = item.box.offsetHeight;
        item.spot = null;
      }
    }
  }

  private hide(item: Item): void {
    item.box.style.visibility = 'hidden';
    item.path.setAttribute('d', '');
    item.dot.setAttribute('visibility', 'hidden');
    item.spot = null;
  }

  /** `obstacles` are screen rectangles labels must keep clear of, such as the bogies. */
  update(camera: THREE.PerspectiveCamera, width: number, height: number, safe: SafeArea, obstacles: Rect[]): void {
    if (!this.visible) {
      return;
    }
    this.svg.setAttribute('width', String(width));
    this.svg.setAttribute('height', String(height));
    const free: Rect = { x0: safe.left + margin, y0: safe.top + margin, x1: width - safe.right - margin, y1: height - safe.bottom - margin };
    const anchors: { item: Item; x: number; y: number }[] = [];
    for (const item of this.items) {
      item.label.anchor.getWorldPosition(this.world);
      const large = item.label.sizeMeters / Math.max(1e-6, camera.position.distanceTo(this.world)) >= smallestAngle;
      this.world.project(camera);
      const x = ((this.world.x + 1) / 2) * width;
      const y = ((1 - this.world.y) / 2) * height;
      const onScreen = this.world.z < 1 && contains(free, x, y, 0);
      if (large && onScreen) {
        anchors.push({ item, x, y });
      } else {
        this.hide(item);
      }
    }
    const centreX = anchors.reduce((sum, anchor) => sum + anchor.x, 0) / Math.max(1, anchors.length);
    const boxes: Rect[] = [];
    const leaders: Segment[] = [];
    for (const anchor of anchors) {
      const { item } = anchor;
      const preferred: -1 | 1 = anchor.x < centreX ? -1 : 1;
      const fresh: Spot[] = [];
      for (const side of [preferred, -preferred as -1 | 1]) {
        for (const dx of distances) {
          for (const dy of heights) {
            fresh.push({ side, dx, dy });
          }
        }
      }
      // Shortest leader first, with a small preference for the anchor's own side.
      const cost = (spot: Spot): number => Math.hypot(spot.dx, spot.dy) + (spot.side === preferred ? 0 : 70);
      fresh.sort((a, b) => cost(a) - cost(b));
      const candidates: Spot[] = item.spot === null ? fresh : [item.spot, ...fresh];
      let placed: { spot: Spot; rect: Rect; legs: Segment[] } | null = null;
      for (const spot of candidates) {
        const y = anchor.y + spot.dy;
        const edge = anchor.x + spot.side * spot.dx;
        const rect: Rect =
          spot.side > 0
            ? { x0: edge, y0: y - item.height / 2, x1: edge + item.width, y1: y + item.height / 2 }
            : { x0: edge - item.width, y0: y - item.height / 2, x1: edge, y1: y + item.height / 2 };
        if (rect.x0 < free.x0 || rect.x1 > free.x1 || rect.y0 < free.y0 || rect.y1 > free.y1) {
          continue;
        }
        if (boxes.some((box) => overlaps(box, rect, margin)) || obstacles.some((obstacle) => overlaps(obstacle, rect, 4))) {
          continue;
        }
        if (anchors.some((other) => contains(rect, other.x, other.y, 8))) {
          continue;
        }
        const elbowX = edge - spot.side * 16;
        const legs: Segment[] = [
          { ax: anchor.x, ay: anchor.y, bx: elbowX, by: y },
          { ax: elbowX, ay: y, bx: edge - spot.side * 4, by: y },
        ];
        if (legs.some((leg) => boxes.some((box) => crosses(leg, box)))) {
          continue;
        }
        if (leaders.some((leader) => crosses(leader, rect) || legs.some((leg) => segmentsIntersect(leg, leader)))) {
          continue;
        }
        placed = { spot, rect, legs };
        break;
      }
      if (placed === null) {
        this.hide(item);
        continue;
      }
      item.spot = placed.spot;
      boxes.push(placed.rect);
      leaders.push(...placed.legs);
      item.box.style.visibility = 'visible';
      item.box.style.transform = `translate(${placed.rect.x0.toFixed(1)}px, ${placed.rect.y0.toFixed(1)}px)`;
      item.box.classList.toggle('left', placed.spot.side < 0);
      const [first, second] = placed.legs as [Segment, Segment];
      item.path.setAttribute(
        'd',
        `M${first.ax.toFixed(1)},${first.ay.toFixed(1)} L${first.bx.toFixed(1)},${first.by.toFixed(1)} L${second.bx.toFixed(1)},${second.by.toFixed(1)}`,
      );
      item.dot.setAttribute('visibility', 'visible');
      item.dot.setAttribute('x', (anchor.x - 3.5).toFixed(1));
      item.dot.setAttribute('y', (anchor.y - 3.5).toFixed(1));
    }
  }

  dispose(): void {
    this.element.remove();
  }
}
