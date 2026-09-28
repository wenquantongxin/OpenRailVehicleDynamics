import type { SceneRecord } from '../record/scene_record.ts';
import { ScalarStatus } from '../record/scene_record.ts';
import type { VehicleDisplayBindings } from '../scene/vehicle_display_bindings.ts';
import { BilingualLabel, Card, DivergingBar } from './Card.tsx';
import { fixed, signed } from './format.ts';
import type { CarrierReadoutBinding, ContactState, Reading, TrackBodyReadout, VehicleReadoutModel, WheelReadout } from './vehicle_readout_model.ts';

// Cards 01, 03 and 04: carriers, vehicle, vertical wheel–rail forces. Every
// number is a recorded sample at the displayed frame; every name and every
// grouping comes from the display bindings.

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

/** A reading in display units, the recorded status when it is not valid, or "not recorded" when the record never exported it. */
export function shown(reading: Reading | null, scale: number, digits: number, withSign: boolean): string {
  if (reading === null) {
    return 'Not recorded 未记录';
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
function halfDatumSpacingOf(record: SceneRecord): number | null {
  const left = record.track?.leftRailDatumInInertialMeters[0];
  const right = record.track?.rightRailDatumInInertialMeters[0];
  if (left === undefined || right === undefined) {
    return null;
  }
  return 0.5 * Math.hypot((right[0] ?? 0) - (left[0] ?? 0), (right[1] ?? 0) - (left[1] ?? 0), (right[2] ?? 0) - (left[2] ?? 0));
}

// The plan uses one true scale in both directions: nothing is magnified, so
// millimetre offsets and milliradian yaw stay as small as they really are.
// The carbody span between the bogies is broken out with a drafting break.
const sleeperHalfLength = 1.3;
const sleeperSpacing = 0.6;
const sleeperWidth = 0.26;
const beamWidth = 0.2;

interface PlanPanel {
  centreOffsetMeters: number;
  carriers: CarrierReadoutBinding[];
}

/**
 * Panels of the carrier plan: one per bogie that groups carriers, centred on
 * the bogie frame, plus one for carriers no bogie lists. null when a
 * longitudinal reference is missing, which happens without a carbody binding.
 */
function planPanels(readoutModel: VehicleReadoutModel): PlanPanel[] | null {
  const panels: PlanPanel[] = [];
  for (const [bogieIndex, bogie] of readoutModel.bogies.entries()) {
    const members = readoutModel.carriers.filter((carrier) => carrier.bogieIndex === bogieIndex);
    if (members.length === 0) {
      continue;
    }
    if (bogie.initialLongitudinalOffsetInCarbodyFrameMeters === null) {
      return null;
    }
    panels.push({ centreOffsetMeters: bogie.initialLongitudinalOffsetInCarbodyFrameMeters, carriers: members });
  }
  const ungrouped = readoutModel.carriers.filter((carrier) => carrier.bogieIndex === null);
  if (ungrouped.length > 0) {
    let sum = 0;
    for (const carrier of ungrouped) {
      if (carrier.initialLongitudinalOffsetInCarbodyFrameMeters === null) {
        return null;
      }
      sum += carrier.initialLongitudinalOffsetInCarbodyFrameMeters;
    }
    panels.push({ centreOffsetMeters: sum / ungrouped.length, carriers: ungrouped });
  }
  if (panels.some((panel) => panel.carriers.some((carrier) => carrier.initialLongitudinalOffsetInCarbodyFrameMeters === null))) {
    return null;
  }
  // Left to right is rear to front, as the forward arrow says; this is layout, not identity.
  panels.sort((a, b) => a.centreOffsetMeters - b.centreOffsetMeters);
  return panels;
}

function CarrierPlan(props: { record: SceneRecord; readoutModel: VehicleReadoutModel; frameIndex: number; wheelWidth: number | null; panels: PlanPanel[] }) {
  const { record, readoutModel, frameIndex, panels } = props;
  const width = 236;
  const height = 124;
  const top = 20;
  const bottom = height - 14;
  const centreY = 0.5 * (top + bottom);
  const leftMargin = 24;
  const rightMargin = 4;
  const gap = 26;
  const placements = record.wheelPlacements;
  const halfSpacing = halfDatumSpacingOf(record) ?? Math.max(0.5, ...placements.map((p) => Math.abs(p.datumInWheelBodyFrameMeters[1])));
  const radius = Math.max(0.1, ...placements.map((p) => p.nominalRollingRadiusMeters));
  const wheelWidth = props.wheelWidth ?? 0.3 * radius;
  const offsetOf = (carrier: CarrierReadoutBinding): number => carrier.initialLongitudinalOffsetInCarbodyFrameMeters ?? 0;
  const sized = panels.map((panel) => ({
    ...panel,
    half: Math.max(...panel.carriers.map((carrier) => Math.abs(offsetOf(carrier) - panel.centreOffsetMeters))) + radius + 0.35,
  }));
  const scale = Math.min(
    (width - leftMargin - rightMargin - gap * Math.max(0, sized.length - 1)) / sized.reduce((sum, panel) => sum + 2 * panel.half, 0),
    (bottom - top) / (2 * (sleeperHalfLength + 0.08)),
  );
  let cursor = leftMargin;
  const laid = sized.map((panel) => {
    const x0 = cursor;
    cursor += 2 * panel.half * scale + gap;
    return { ...panel, x0, x1: x0 + 2 * panel.half * scale, xOf: (offset: number) => x0 + (offset - panel.centreOffsetMeters + panel.half) * scale };
  });
  const railTop = centreY - halfSpacing * scale;
  const railBottom = centreY + halfSpacing * scale;
  const bandTop = centreY - (sleeperHalfLength + 0.06) * scale;
  const bandBottom = centreY + (sleeperHalfLength + 0.06) * scale;
  const lateralOf = (wheel: WheelReadout | null): number => {
    const placement = wheel === null ? undefined : placements[wheel.placementIndex];
    const magnitude = placement === undefined ? halfSpacing : Math.abs(placement.datumInWheelBodyFrameMeters[1]);
    return (wheel?.side === 'left' ? -1 : 1) * magnitude;
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
    <svg className="schematic" width={width} height={height} viewBox={`0 0 ${width} ${height}`} aria-label="Carrier plan, true scale">
      {laid.map((panel) => {
        const sleepers = [];
        for (let k = Math.ceil((-panel.half + 0.15) / sleeperSpacing); k * sleeperSpacing <= panel.half - 0.15; ++k) {
          const x = panel.xOf(panel.centreOffsetMeters + k * sleeperSpacing);
          sleepers.push(
            <rect key={k} x={x - 0.5 * sleeperWidth * scale} y={centreY - sleeperHalfLength * scale} width={sleeperWidth * scale} height={2 * sleeperHalfLength * scale} className="sch-sleeper" />,
          );
        }
        return (
          <g key={panel.centreOffsetMeters}>
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
          <g key={`break-${panel.centreOffsetMeters}`}>
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
        panel.carriers.map((carrier) => {
          // The drawn shift and turn are the recorded lateral scalar and the
          // heading from the recorded pose against the local track frame
          // (positive towards the right rail, i.e. clockwise here). A carrier
          // whose lateral position was not recorded is drawn at the nominal
          // place in the "unrecorded" style, not as a zero measurement. The
          // recorded yaw scalar is shown unchanged in the table.
          const lateral = readoutModel.read(frameIndex, carrier.lateralScalar);
          const recorded = lateral?.valid === true;
          const heading = recorded ? readoutModel.headingAt(frameIndex, carrier) : null;
          const x = panel.xOf(offsetOf(carrier));
          const angle = ((heading ?? 0) * 180) / Math.PI;
          const shift = recorded ? (lateral?.value ?? 0) * scale : 0;
          const leftY = lateralOf(carrier.leftWheel) * scale;
          const rightY = lateralOf(carrier.rightWheel) * scale;
          const wheel = (readout: WheelReadout | null, y: number) => {
            const patches = readoutModel.read(frameIndex, readout?.contactPatchCountScalar ?? null);
            const kind = patches?.valid !== true ? 'unknown' : patches.value === 0 ? 'loss' : patches.value >= 2 ? 'two' : 'single';
            return (
              <rect x={-radius * scale} y={y - 0.5 * wheelWidth * scale} width={2 * radius * scale} height={wheelWidth * scale} rx={0.6} className={`sch-wheel ${kind}`} />
            );
          };
          return (
            <g key={carrier.number}>
              <g transform={`translate(${x.toFixed(2)} ${(centreY + shift).toFixed(3)}) rotate(${angle.toFixed(4)})`}>
                <line x1={0} y1={leftY} x2={0} y2={rightY} className={`sch-axle ${recorded ? '' : 'unrecorded'}`} style={{ strokeWidth: beamWidth * scale }} />
                {wheel(carrier.leftWheel, leftY)}
                {wheel(carrier.rightWheel, rightY)}
              </g>
              <text x={x} y={12} textAnchor="middle" className="sch-number">{carrier.number}</text>
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

export function CarriersCard(props: { record: SceneRecord; readoutModel: VehicleReadoutModel; frameIndex: number; wheelWidth: number | null }) {
  const { readoutModel, frameIndex } = props;
  const contact = readoutModel.contactAt(frameIndex);
  const name = contactNames[contact.state];
  const coverage = `${contact.known} / ${contact.total}`;
  const detail =
    contact.state === 'two' || contact.state === 'loss'
      ? contact.known < contact.total
        ? `${contact.codes.join(' ')} · ${coverage}`
        : contact.codes.join(' ')
      : coverage;
  const panels = planPanels(readoutModel);
  return (
    <Card index="01" en="Carriers" zh="载体">
      <div className="status">
        <span className="status-chip" style={{ background: name.color }} />
        <span className="status-text">
          <span className="en">{name.en}</span>
          <span className="zh">{name.zh}</span>
        </span>
        <span className="status-codes">{detail}</span>
      </div>
      {panels !== null && panels.length > 0 ? (
        <>
          <CarrierPlan record={props.record} readoutModel={readoutModel} frameIndex={frameIndex} wheelWidth={props.wheelWidth} panels={panels} />
          <p className="caption">
            PLAN · TRUE SCALE · MID-SPAN OMITTED
            <span className="zh">俯视原比例，转向架间断开省略；转角取自记录位姿，未记录横移的载体按标称位置虚线画出</span>
          </p>
        </>
      ) : (
        <p className="caption">
          PLAN UNAVAILABLE · NO CARBODY REFERENCE
          <span className="zh">无车体参考，俯视示意不可用</span>
        </p>
      )}
      <div className="table">
        <div className="table-head">
          <BilingualLabel en="No." zh="序号" />
          <BilingualLabel en="Lateral" unit="mm" zh="横移，右正" />
          <BilingualLabel en="Yaw" unit="mrad" zh="摇头角" title={definitionText(props.record, readoutModel.carriers[0]?.yawScalar ?? null)} />
        </div>
        {readoutModel.carriers.map((carrier) => (
          <div className="table-row" key={carrier.number}>
            <span className="axle-number" title={`${carrier.displayName.en} ${carrier.displayName.zh} (${carrier.bodyName})`}>
              {carrier.number}
            </span>
            <b>{shown(readoutModel.read(frameIndex, carrier.lateralScalar), 1000, 1, true)}</b>
            <b>{shown(readoutModel.read(frameIndex, carrier.yawScalar), 1000, 2, true)}</b>
          </div>
        ))}
      </div>
      <p className="caption">
        {readoutModel.carriers.map((carrier) => `${carrier.number} ${carrier.displayName.en}`).join(' · ')}
        <span className="zh">{readoutModel.carriers.map((carrier) => `${carrier.number} ${carrier.displayName.zh}`).join(' · ')}</span>
      </p>
    </Card>
  );
}

function BodyRows({ body, readoutModel, frameIndex, yawTitle }: { body: TrackBodyReadout; readoutModel: VehicleReadoutModel; frameIndex: number; yawTitle: string | undefined }) {
  const lateral = readoutModel.read(frameIndex, body.lateralScalar);
  const yaw = readoutModel.read(frameIndex, body.yawScalar);
  return (
    <div className="body-block">
      <BilingualLabel en={body.displayName.en} zh={body.displayName.zh} className="body-name" />
      <div className="brow">
        <span className="q">LAT</span>
        <DivergingBar value={numeric(lateral, 1000)} scale={readoutModel.scales.bodyLateralMm} />
        <b>{shown(lateral, 1000, 1, true)} <small>mm</small></b>
      </div>
      <div className="brow">
        <span className="q" title={yawTitle}>
          YAW
        </span>
        <DivergingBar value={numeric(yaw, 1000)} scale={readoutModel.scales.bodyYawMrad} />
        <b>{shown(yaw, 1000, 2, true)} <small>mrad</small></b>
      </div>
    </div>
  );
}

export function VehicleCard({ record, readoutModel, bindings, frameIndex }: { record: SceneRecord; readoutModel: VehicleReadoutModel; bindings: VehicleDisplayBindings; frameIndex: number }) {
  const bodies = [readoutModel.carbody, ...readoutModel.bogies].filter((body): body is TrackBodyReadout => body !== null);
  return (
    <Card index="03" en="Vehicle" zh="车辆">
      <div className="stats">
        <div className="stat">
          <span className="n">{readoutModel.wheels.length}</span>
          <BilingualLabel en="Wheels" zh="车轮" />
        </div>
        <div className="stat">
          <span className="n">{bindings.bogies.length}</span>
          <BilingualLabel en="Bogies" zh="转向架" />
        </div>
        <div className="stat">
          <span className="n">{record.bodies.length}</span>
          <BilingualLabel en="Bodies" zh="刚体" />
        </div>
      </div>
      <p className="caption">
        RELATIVE TO TRACK · ±{readoutModel.scales.bodyLateralMm} mm · ±{readoutModel.scales.bodyYawMrad} mrad
        <span className="zh">相对线路，横移以右为正</span>
      </p>
      {bodies.length === 0 ? (
        <p className="caption">
          NO CARBODY OR BOGIE BINDING
          <span className="zh">视觉定义未绑定车体与构架，无相应读数</span>
        </p>
      ) : (
        bodies.map((body) => (
          <BodyRows key={body.bodyName} body={body} readoutModel={readoutModel} frameIndex={frameIndex} yawTitle={definitionText(record, body.yawScalar)} />
        ))
      )}
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

export function WheelForceCard({ readoutModel, frameIndex }: { readoutModel: VehicleReadoutModel; frameIndex: number }) {
  const scale = readoutModel.scales.forceKn;
  const bar = (wheel: WheelReadout | null, side: 'left' | 'right') => {
    const force = numeric(readoutModel.read(frameIndex, wheel?.verticalSupportForceScalar ?? null), 0.001);
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
        <BilingualLabel en="Left" unit="kN" zh="左轮" />
        <BilingualLabel en="Right" unit="kN" zh="右轮" className="right" />
      </div>
      {readoutModel.carriers.map((carrier) => (
        <div className="bf-row" key={carrier.number}>
          <b className="bf-value">{shown(readoutModel.read(frameIndex, carrier.leftWheel?.verticalSupportForceScalar ?? null), 0.001, 1, false)}</b>
          {bar(carrier.leftWheel, 'left')}
          <span className="bf-axle">
            <ContactDots reading={readoutModel.read(frameIndex, carrier.leftWheel?.contactPatchCountScalar ?? null)} />
            <b>{carrier.number}</b>
            <ContactDots reading={readoutModel.read(frameIndex, carrier.rightWheel?.contactPatchCountScalar ?? null)} />
          </span>
          {bar(carrier.rightWheel, 'right')}
          <b className="bf-value right">{shown(readoutModel.read(frameIndex, carrier.rightWheel?.verticalSupportForceScalar ?? null), 0.001, 1, false)}</b>
        </div>
      ))}
      <p className="caption">
        VERTICAL WHEEL–RAIL FORCE · SCALE {scale} kN
        <span className="zh">轮轨垂向力；圆点为接触点数，空心为轮轨分离</span>
      </p>
    </Card>
  );
}
