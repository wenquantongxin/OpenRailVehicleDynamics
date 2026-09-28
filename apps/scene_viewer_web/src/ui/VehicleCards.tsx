import type { SceneRecord } from '../record/scene_record.ts';
import { ScalarStatus } from '../record/scene_record.ts';
import { Bi, Card, DivergingBar } from './Card.tsx';
import { fixed, signed } from './format.ts';
import type { ContactState, Reading, ReadoutModel, TrackBodyRef, WheelRef } from './readout_model.ts';

// Cards 01, 03 and 04: axle bridges, vehicle, vertical wheel–rail forces.
// Every number is a recorded sample at the displayed frame.

export const contactNames: Record<ContactState, { en: string; zh: string; color: string }> = {
  single: { en: 'Single-point contact', zh: '单点接触', color: 'var(--ok)' },
  two: { en: 'Two-point contact', zh: '两点接触', color: 'var(--accent)' },
  loss: { en: 'Loss of contact', zh: '轮轨分离', color: 'var(--alert)' },
  incomplete: { en: 'Contact partly not ready', zh: '部分接触未就绪', color: 'var(--ink3)' },
  unknown: { en: 'Contact not ready', zh: '接触未就绪', color: 'var(--ink3)' },
};

/** The recorded definition of a scalar, for a header tooltip. */
function definitionText(record: SceneRecord, ref: { index: number } | null): string | undefined {
  const definition = ref === null ? undefined : record.scalars[ref.index];
  return definition === undefined ? undefined : `${definition.quantity}\n${definition.referenceFrame}\n${definition.method}`;
}

/** A reading in display units, or the recorded status when it is not valid. */
export function shown(reading: Reading | null, scale: number, digits: number, withSign: boolean): string {
  if (reading === null) {
    return '—';
  }
  if (!reading.valid) {
    return reading.status === ScalarStatus.placeholder ? 'Placeholder 占位' : 'Not ready 未就绪';
  }
  return withSign ? signed(reading.value * scale, digits) : fixed(reading.value * scale, digits);
}

function numeric(reading: Reading | null, scale: number): number | null {
  return reading !== null && reading.valid ? reading.value * scale : null;
}

/** Half the distance between the two recorded rail datums. */
function halfGaugeOf(record: SceneRecord): number | null {
  const left = record.track?.leftRailDatumInInertialMeters[0];
  const right = record.track?.rightRailDatumInInertialMeters[0];
  if (left === undefined || right === undefined) {
    return null;
  }
  return 0.5 * Math.hypot((right[0] ?? 0) - (left[0] ?? 0), (right[1] ?? 0) - (left[1] ?? 0), (right[2] ?? 0) - (left[2] ?? 0));
}

function nearest(values: number[], target: number): number {
  return values.reduce((best, value) => (Math.abs(value - target) < Math.abs(best - target) ? value : best), values[0] ?? 0);
}

// The plan uses one true scale in both directions: nothing is magnified, so
// millimetre offsets and milliradian yaw stay as small as they really are.
// The carbody span between the bogies is broken out with a drafting break.
const sleeperHalfLength = 1.3;
const sleeperSpacing = 0.6;
const sleeperWidth = 0.26;
const beamWidth = 0.2;

function AxleSchematic(props: { record: SceneRecord; model: ReadoutModel; frame: number; wheelWidth: number | null }) {
  const { record, model, frame } = props;
  const width = 236;
  const height = 124;
  const top = 20;
  const bottom = height - 14;
  const centreY = 0.5 * (top + bottom);
  const leftMargin = 24;
  const rightMargin = 4;
  const gap = 26;
  const placements = record.wheelPlacements;
  const halfGauge = halfGaugeOf(record) ?? Math.max(0.5, ...placements.map((p) => Math.abs(p.datumInWheelBodyFrameMeters[1])));
  const radius = Math.max(0.1, ...placements.map((p) => p.nominalRollingRadiusMeters));
  const wheelWidth = props.wheelWidth ?? 0.3 * radius;
  const centres =
    model.bogies.length > 0
      ? model.bogies.map((bogie) => bogie.offset)
      : [model.axles.reduce((sum, axle) => sum + axle.offset, 0) / Math.max(1, model.axles.length)];
  const panels = centres
    .map((centre) => ({ centre, axles: model.axles.filter((axle) => nearest(centres, axle.offset) === centre) }))
    .filter((panel) => panel.axles.length > 0)
    .sort((a, b) => a.centre - b.centre)
    .map((panel) => ({ ...panel, half: Math.max(...panel.axles.map((axle) => Math.abs(axle.offset - panel.centre))) + radius + 0.35 }));
  const scale = Math.min(
    (width - leftMargin - rightMargin - gap * Math.max(0, panels.length - 1)) / panels.reduce((sum, panel) => sum + 2 * panel.half, 0),
    (bottom - top) / (2 * (sleeperHalfLength + 0.08)),
  );
  let cursor = leftMargin;
  const laid = panels.map((panel) => {
    const x0 = cursor;
    cursor += 2 * panel.half * scale + gap;
    return { ...panel, x0, x1: x0 + 2 * panel.half * scale, xOf: (offset: number) => x0 + (offset - panel.centre + panel.half) * scale };
  });
  const railTop = centreY - halfGauge * scale;
  const railBottom = centreY + halfGauge * scale;
  const bandTop = centreY - (sleeperHalfLength + 0.06) * scale;
  const bandBottom = centreY + (sleeperHalfLength + 0.06) * scale;
  const lateralOf = (ref: WheelRef | null): number => {
    const placement = ref === null ? undefined : placements[ref.placement];
    const magnitude = placement === undefined ? halfGauge : Math.abs(placement.datumInWheelBodyFrameMeters[1]);
    return (ref?.side === 'left' ? -1 : 1) * magnitude;
  };
  const zigzag = (x: number): string => {
    const points: string[] = [];
    let flip = -1;
    for (let y = bandTop; y <= bandBottom + 0.01; y += 5) {
      points.push(`${(x + flip * 2.2).toFixed(1)},${Math.min(y, bandBottom).toFixed(1)}`);
      flip = -flip;
    }
    return `M${points.join(' L')}`;
  };
  return (
    <svg className="schematic" width={width} height={height} viewBox={`0 0 ${width} ${height}`} aria-label="Axle bridge plan, true scale">
      {laid.map((panel) => {
        const sleepers = [];
        for (let k = Math.ceil((-panel.half + 0.15) / sleeperSpacing); k * sleeperSpacing <= panel.half - 0.15; ++k) {
          const x = panel.xOf(panel.centre + k * sleeperSpacing);
          sleepers.push(
            <rect key={k} x={x - 0.5 * sleeperWidth * scale} y={centreY - sleeperHalfLength * scale} width={sleeperWidth * scale} height={2 * sleeperHalfLength * scale} className="sch-sleeper" />,
          );
        }
        return (
          <g key={panel.centre}>
            {sleepers}
            <line x1={panel.x0} y1={railTop} x2={panel.x1} y2={railTop} className="sch-rail" />
            <line x1={panel.x0} y1={railBottom} x2={panel.x1} y2={railBottom} className="sch-rail" />
          </g>
        );
      })}
      {laid.slice(1).map((panel, index) => {
        const previous = laid[index];
        const x = previous === undefined ? panel.x0 - gap / 2 : 0.5 * (previous.x1 + panel.x0);
        return (
          <g key={`break-${panel.centre}`}>
            <path d={zigzag(x - 2.5)} className="sch-break" />
            <path d={zigzag(x + 2.5)} className="sch-break" />
          </g>
        );
      })}
      <text x={2} y={railTop + 3} className="sch-side">
        L <tspan className="zh">左</tspan>
      </text>
      <text x={2} y={railBottom + 3} className="sch-side">
        R <tspan className="zh">右</tspan>
      </text>
      {laid.flatMap((panel) =>
        panel.axles.map((axle) => {
          const lateral = numeric(model.read(frame, axle.bridge?.lateral ?? null), 1) ?? 0;
          // The drawn turn comes from the recorded pose against the local track
          // frame (positive towards the right rail, i.e. clockwise here). The
          // recorded yaw scalar is an X-Z-Y angle in the body basis and is shown
          // unchanged in the table; its sign depends on the body basis.
          const heading = axle.bridge === null ? null : model.headingAt(frame, axle.bridge);
          const x = panel.xOf(axle.offset);
          const angle = ((heading ?? 0) * 180) / Math.PI;
          const leftY = lateralOf(axle.left) * scale;
          const rightY = lateralOf(axle.right) * scale;
          const wheel = (ref: WheelRef | null, y: number) => {
            const patches = model.read(frame, ref?.patches ?? null);
            const kind =
              patches?.valid !== true ? 'unknown' : patches.value === 0 ? 'loss' : patches.value >= 2 ? 'two' : 'single';
            return (
              <rect x={-radius * scale} y={y - 0.5 * wheelWidth * scale} width={2 * radius * scale} height={wheelWidth * scale} rx={0.6} className={`sch-wheel ${kind}`} />
            );
          };
          return (
            <g key={axle.number}>
              <g transform={`translate(${x.toFixed(2)} ${(centreY + lateral * scale).toFixed(3)}) rotate(${angle.toFixed(4)})`}>
                <line x1={0} y1={leftY} x2={0} y2={rightY} className="sch-axle" style={{ strokeWidth: beamWidth * scale }} />
                {wheel(axle.left, leftY)}
                {wheel(axle.right, rightY)}
              </g>
              <text x={x} y={12} textAnchor="middle" className="sch-number">{axle.number}</text>
            </g>
          );
        }),
      )}
      <line x1={leftMargin} y1={height - 5} x2={leftMargin + scale} y2={height - 5} className="sch-scale" />
      <line x1={leftMargin} y1={height - 8} x2={leftMargin} y2={height - 2} className="sch-scale" />
      <line x1={leftMargin + scale} y1={height - 8} x2={leftMargin + scale} y2={height - 2} className="sch-scale" />
      <text x={leftMargin + scale + 4} y={height - 2.5} className="sch-text">1 m</text>
      <text x={width - 2} y={height - 2.5} textAnchor="end" className="sch-text">FORWARD 前向 →</text>
    </svg>
  );
}

export function AxleBridgeCard(props: { record: SceneRecord; model: ReadoutModel; frame: number; wheelWidth: number | null }) {
  const { model, frame } = props;
  const contact = model.contactAt(frame);
  const name = contactNames[contact.state];
  const coverage = `${contact.known} / ${contact.total}`;
  const detail =
    contact.state === 'two' || contact.state === 'loss'
      ? contact.known < contact.total
        ? `${contact.codes.join(' ')} · ${coverage}`
        : contact.codes.join(' ')
      : coverage;
  return (
    <Card index="01" en="Axle bridges" zh="轴桥">
      <div className="status">
        <span className="status-chip" style={{ background: name.color }} />
        <span className="status-text">
          <span className="en">{name.en}</span>
          <span className="zh">{name.zh}</span>
        </span>
        <span className="status-codes">{detail}</span>
      </div>
      <AxleSchematic record={props.record} model={model} frame={frame} wheelWidth={props.wheelWidth} />
      <p className="caption">
        PLAN · TRUE SCALE · MID-SPAN OMITTED
        <span className="zh">俯视原比例，转向架间断开省略；转角取自记录位姿</span>
      </p>
      <div className="table">
        <div className="table-head">
          <Bi en="Axle" zh="轴位" />
          <Bi en="Lateral" unit="mm" zh="横移，右正" />
          <Bi en="Yaw" unit="mrad" zh="摇头角" title={definitionText(props.record, model.axles[0]?.bridge?.yaw ?? null)} />
        </div>
        {model.axles.map((axle) => (
          <div className="table-row" key={axle.number}>
            <span className="axle-number">{axle.number}</span>
            <b>{shown(model.read(frame, axle.bridge?.lateral ?? null), 1000, 1, true)}</b>
            <b>{shown(model.read(frame, axle.bridge?.yaw ?? null), 1000, 2, true)}</b>
          </div>
        ))}
      </div>
    </Card>
  );
}

function BodyRows({ body, model, frame, yawTitle }: { body: TrackBodyRef; model: ReadoutModel; frame: number; yawTitle: string | undefined }) {
  const lateral = model.read(frame, body.lateral);
  const yaw = model.read(frame, body.yaw);
  return (
    <div className="body-block">
      <Bi en={body.en} zh={body.zh} className="body-name" />
      <div className="brow">
        <span className="q">LAT</span>
        <DivergingBar value={numeric(lateral, 1000)} scale={model.scales.bodyLateralMm} />
        <b>{shown(lateral, 1000, 1, true)} <small>mm</small></b>
      </div>
      <div className="brow">
        <span className="q" title={yawTitle}>
          YAW
        </span>
        <DivergingBar value={numeric(yaw, 1000)} scale={model.scales.bodyYawMrad} />
        <b>{shown(yaw, 1000, 2, true)} <small>mrad</small></b>
      </div>
    </div>
  );
}

export function VehicleCard({ record, model, frame }: { record: SceneRecord; model: ReadoutModel; frame: number }) {
  const bodies = [model.carbody, ...model.bogies].filter((body): body is TrackBodyRef => body !== null);
  return (
    <Card index="03" en="Vehicle" zh="车辆">
      <div className="stats">
        <div className="stat">
          <span className="n">{model.wheels.length}</span>
          <Bi en="Wheels" zh="车轮" />
        </div>
        <div className="stat">
          <span className="n">{model.bogies.length}</span>
          <Bi en="Bogies" zh="转向架" />
        </div>
        <div className="stat">
          <span className="n">{record.bodies.length}</span>
          <Bi en="Bodies" zh="刚体" />
        </div>
      </div>
      <p className="caption">
        RELATIVE TO TRACK · ±{model.scales.bodyLateralMm} mm · ±{model.scales.bodyYawMrad} mrad
        <span className="zh">相对线路，横移以右为正</span>
      </p>
      {bodies.map((body) => (
        <BodyRows key={body.bodyName} body={body} model={model} frame={frame} yawTitle={definitionText(record, body.yaw)} />
      ))}
    </Card>
  );
}

function ContactDots({ reading }: { reading: Reading | null }) {
  if (reading === null) {
    return <span className="dots" />;
  }
  if (!reading.valid) {
    return (
      <span className="dots" title="Not ready 未就绪">
        <i className="unknown" />
      </span>
    );
  }
  if (reading.value === 0) {
    return (
      <span className="dots">
        <i className="loss" />
      </span>
    );
  }
  return (
    <span className="dots">
      {Array.from({ length: Math.min(3, reading.value) }, (_, index) => (
        <i key={index} className={reading.value >= 2 ? 'two' : ''} />
      ))}
    </span>
  );
}

export function WheelForceCard({ model, frame }: { model: ReadoutModel; frame: number }) {
  const scale = model.scales.forceKn;
  const bar = (wheel: WheelRef | null, side: 'left' | 'right') => {
    const force = numeric(model.read(frame, wheel?.force ?? null), 0.001);
    const width = force === null ? 0 : Math.max(0, Math.min(100, (100 * force) / scale));
    return (
      <span className={`bf-bar ${side}`}>
        <i style={{ width: `${width}%` }} />
      </span>
    );
  };
  return (
    <Card index="04" en="Wheel loads" zh="轮重">
      <div className="bf-head">
        <Bi en="Left" unit="kN" zh="左轮" />
        <Bi en="Right" unit="kN" zh="右轮" className="right" />
      </div>
      {model.axles.map((axle) => (
        <div className="bf-row" key={axle.number}>
          <b className="bf-value">{shown(model.read(frame, axle.left?.force ?? null), 0.001, 1, false)}</b>
          {bar(axle.left, 'left')}
          <span className="bf-axle">
            <ContactDots reading={model.read(frame, axle.left?.patches ?? null)} />
            <b>{axle.number}</b>
            <ContactDots reading={model.read(frame, axle.right?.patches ?? null)} />
          </span>
          {bar(axle.right, 'right')}
          <b className="bf-value right">{shown(model.read(frame, axle.right?.force ?? null), 0.001, 1, false)}</b>
        </div>
      ))}
      <p className="caption">
        VERTICAL WHEEL–RAIL FORCE · SCALE {scale} kN
        <span className="zh">轮轨垂向力；圆点为接触点数，空心为轮轨分离</span>
      </p>
    </Card>
  );
}
