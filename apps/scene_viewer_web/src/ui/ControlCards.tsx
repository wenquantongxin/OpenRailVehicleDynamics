import type { BilingualName } from '../record/visual_definition.ts';
import type { ViewPreset } from '../scene/camera_rig.ts';
import type { CarbodyMode } from '../scene/scene_builder.ts';
import type { ViewportOptions } from '../viewer/SceneViewport.tsx';
import { Card } from './Card.tsx';

// Cards 05 and 06: camera presets with the camera's own behaviour, and what
// the stage draws. The bogie view carries the bound bogie's own name and is
// disabled when the bindings provide none.

export const viewNames: Record<ViewPreset, { en: string; zh: string; key: string }> = {
  overview: { en: 'Overview', zh: '整车', key: '1' },
  bogie: { en: 'Bogie', zh: '转向架', key: '2' },
  headon: { en: 'Head-on', zh: '迎面', key: '3' },
  plan: { en: 'Plan', zh: '俯视', key: '4' },
  side: { en: 'Side', zh: '侧视', key: '5' },
};

const carbodyModes: { mode: CarbodyMode; en: string; zh: string }[] = [
  { mode: 'xray', en: 'X-ray', zh: '透视' },
  { mode: 'solid', en: 'Solid', zh: '实体' },
  { mode: 'hidden', en: 'Hidden', zh: '隐藏' },
];

function Chip(props: { on: boolean; en: string; zh: string; hint?: string; disabled?: boolean; onClick: () => void }) {
  return (
    <button type="button" className={`chip ${props.on ? 'on' : ''}`} aria-pressed={props.on} disabled={props.disabled === true} onClick={props.onClick} title={props.hint}>
      <span className="en">{props.en}</span>
      <span className="zh">{props.zh}</span>
    </button>
  );
}

export function ViewsCard(props: {
  current: ViewPreset;
  /** Presets whose anchors the bindings provide; the others are shown disabled. */
  available: ReadonlySet<ViewPreset>;
  bogieDisplayName: BilingualName | null;
  onSelect: (preset: ViewPreset) => void;
  options: ViewportOptions;
  onChange: (next: Partial<ViewportOptions>) => void;
}) {
  const { options, onChange } = props;
  return (
    <Card index="05" en="Views" zh="视角">
      <div className="chips">
        {(Object.keys(viewNames) as ViewPreset[]).map((preset) => {
          const names = preset === 'bogie' && props.bogieDisplayName !== null ? props.bogieDisplayName : viewNames[preset];
          const available = props.available.has(preset);
          return (
            <Chip
              key={preset}
              on={preset === props.current}
              en={names.en}
              zh={names.zh}
              hint={available ? `Key ${viewNames[preset].key}` : 'No bogie binding 无构架绑定'}
              disabled={!available}
              onClick={() => props.onSelect(preset)}
            />
          );
        })}
      </div>
      <div className="chips quiet">
        <Chip on={options.follow} en="Follow" zh="跟随" hint="Key F" onClick={() => onChange({ follow: !options.follow })} />
        <Chip on={options.orbit} en="Orbit" zh="环绕" hint="Key O" onClick={() => onChange({ orbit: !options.orbit })} />
      </div>
    </Card>
  );
}

export function DisplayCard(props: {
  options: ViewportOptions;
  onChange: (next: Partial<ViewportOptions>) => void;
  scalarsOpen: boolean;
  onToggleScalars: () => void;
}) {
  const { options, onChange } = props;
  return (
    <Card index="06" en="Display" zh="显示">
      <div className="segmented" role="radiogroup" aria-label="Carbody">
        <span className="segmented-label">
          <span className="en">Carbody</span>
          <span className="zh">车体</span>
        </span>
        {carbodyModes.map((entry) => (
          <button
            key={entry.mode}
            type="button"
            role="radio"
            aria-checked={options.carbody === entry.mode}
            className={options.carbody === entry.mode ? 'on' : ''}
            onClick={() => onChange({ carbody: entry.mode })}
            title="Key X"
          >
            <span className="en">{entry.en}</span>
            <span className="zh">{entry.zh}</span>
          </button>
        ))}
      </div>
      <div className="chips">
        <Chip on={options.labels} en="Labels" zh="标注" hint="Key L" onClick={() => onChange({ labels: !options.labels })} />
        <Chip on={options.track} en="Track" zh="线路" hint="Key T" onClick={() => onChange({ track: !options.track })} />
        <Chip on={options.axes} en="Axes" zh="坐标轴" hint="Key A" onClick={() => onChange({ axes: !options.axes })} />
        <Chip on={props.scalarsOpen} en="Scalars" zh="全部标量" hint="Key S" onClick={props.onToggleScalars} />
      </div>
    </Card>
  );
}
