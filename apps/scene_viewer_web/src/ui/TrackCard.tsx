import { useMemo } from 'react';

import { sectionNames, type TrackModel } from '../scene/track_model.ts';
import { Card, ReadoutRow } from './Card.tsx';
import { chainage, fixed } from './format.ts';
import type { ReadoutModel } from './readout_model.ts';

// Card 02: where the carbody is on the sampled line. The plan is drawn in the
// inertial frame seen from above (x to the right, y downwards on screen).

const mapWidth = 236;
const mapHeight = 118;

/** The line seen from above, turned so its principal axis runs left to right in the direction of increasing station. */
function usePlan(track: TrackModel) {
  return useMemo(() => {
    const points = track.planPoints(4);
    const count = Math.max(1, points.length);
    const meanX = points.reduce((sum, point) => sum + point.x, 0) / count;
    const meanY = points.reduce((sum, point) => sum + point.y, 0) / count;
    let sxx = 0;
    let syy = 0;
    let sxy = 0;
    for (const point of points) {
      sxx += (point.x - meanX) ** 2;
      syy += (point.y - meanY) ** 2;
      sxy += (point.x - meanX) * (point.y - meanY);
    }
    let angle = 0.5 * Math.atan2(2 * sxy, sxx - syy);
    const first = points[0];
    const last = points[points.length - 1];
    if (first !== undefined && last !== undefined) {
      const along = (last.x - first.x) * Math.cos(angle) + (last.y - first.y) * Math.sin(angle);
      if (along < 0) {
        angle += Math.PI;
      }
    }
    const cos = Math.cos(angle);
    const sin = Math.sin(angle);
    // A rotation, never a reflection: screen y stays the inertial right-hand side.
    const turn = (x: number, y: number): [number, number] => [(x - meanX) * cos + (y - meanY) * sin, -(x - meanX) * sin + (y - meanY) * cos];
    // Loops, not spreads: a long line has more points than a call can take as arguments.
    let minU = Infinity;
    let maxU = -Infinity;
    let minV = Infinity;
    let maxV = -Infinity;
    for (const point of points) {
      const [u, v] = turn(point.x, point.y);
      minU = Math.min(minU, u);
      maxU = Math.max(maxU, u);
      minV = Math.min(minV, v);
      maxV = Math.max(maxV, v);
    }
    const scale = Math.min((mapWidth - 20) / Math.max(1e-6, maxU - minU), (mapHeight - 24) / Math.max(1e-6, maxV - minV));
    const offsetU = (mapWidth - scale * (maxU - minU)) / 2;
    const offsetV = (mapHeight - scale * (maxV - minV)) / 2;
    const project = (x: number, y: number): [number, number] => {
      const [u, v] = turn(x, y);
      return [offsetU + (u - minU) * scale, offsetV + (v - minV) * scale];
    };
    const path = points.map((point, index) => `${index === 0 ? 'M' : 'L'}${project(point.x, point.y).map((value) => value.toFixed(1)).join(',')}`).join(' ');
    return { points, project, path };
  }, [track]);
}

export function TrackCard({ track, model, frame }: { track: TrackModel; model: ReadoutModel; frame: number }) {
  const plan = usePlan(track);
  const station = model.carbodyStation[frame] ?? Number.NaN;
  const startStation = model.carbodyStation[0] ?? Number.NaN;
  const known = Number.isFinite(station);
  const section = known ? track.sectionAt(station) : null;
  const curvature = known ? track.curvatureAt(station) : 0;
  const here = known ? track.frameAt(station).position : null;
  const travelled = plan.points.filter((point) => point.station >= startStation && point.station <= station);
  const travelledPath = travelled
    .map((point, index) => `${index === 0 ? 'M' : 'L'}${plan.project(point.x, point.y).map((v) => v.toFixed(1)).join(',')}`)
    .join(' ');
  const [hx, hy] = here === null ? [0, 0] : plan.project(here.x, here.y);
  const radius =
    section === null || section.kind === 'tangent'
      ? '∞'
      : section.kind === 'circular' && section.radiusMeters !== null
        ? `${fixed(section.radiusMeters, 0)} m`
        : Math.abs(curvature) > 0
          ? `${fixed(1 / Math.abs(curvature), 0)} m`
          : '∞';
  const hand = section === null || section.direction === 0 ? '' : section.direction > 0 ? ' · right-hand 右转' : ' · left-hand 左转';
  const speed = model.speedKmhAt(frame);
  return (
    <Card index="02" en="Track" zh="线路">
      <svg className="plan" width={mapWidth} height={mapHeight} viewBox={`0 0 ${mapWidth} ${mapHeight}`} aria-label="Line plan">
        <path d={plan.path} className="plan-line" />
        <path d={travelledPath} className="plan-travelled" />
        {track.elementPoints.map((point) => {
          const at = track.frameAt(point.station).position;
          const [x, y] = plan.project(at.x, at.y);
          return <circle key={`${point.code}-${point.station}`} cx={x} cy={y} r={2.2} className="plan-element" />;
        })}
        {here !== null && <circle cx={hx} cy={hy} r={4.6} className="plan-here" />}
      </svg>
      <ReadoutRow en="Chainage" zh="里程" value={known ? chainage(station) : '—'} />
      <ReadoutRow
        en="Section"
        zh="区段"
        value={
          section === null ? (
            '—'
          ) : (
            <span className="bi inline">
              <span className="en">{sectionNames[section.kind].en}</span>
              <span className="zh">{sectionNames[section.kind].zh}</span>
            </span>
          )
        }
      />
      <ReadoutRow
        en={section?.kind === 'transition' ? 'Local radius' : 'Curve radius'}
        zh={section?.kind === 'transition' ? '当前半径' : '曲线半径'}
        value={
          <>
            {radius}
            <small>{hand}</small>
          </>
        }
      />
      <ReadoutRow en="Cant" zh="超高" value={known ? `${fixed(1000 * track.cantAt(station), 1)} mm` : '—'} />
      <ReadoutRow en="Vehicle speed" zh="车速" value={speed === null ? '—' : `${fixed(speed, 1)} km/h`} />
    </Card>
  );
}
