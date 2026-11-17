import type { BilingualTitle } from '../service/vehicle_modeling_service_protocol.ts';
import { BilingualTitleText } from './BilingualTitleText.tsx';

// The head of the property panel for selected settings: the row's title, under
// the title of the row it stands in. Settings are neither removed nor renamed,
// so the head offers neither.

export function SelectedSettingsHead(props: { title: BilingualTitle; within: BilingualTitle | null }) {
  return (
    <div className="selection-head settings-head">
      <div className="selection-title">
        {props.within !== null && (
          <span className="selection-kind">
            <BilingualTitleText title={props.within} />
          </span>
        )}
        <span className="selection-name settings-title">
          <BilingualTitleText title={props.title} />
        </span>
      </div>
    </div>
  );
}
