import type { ElementReference } from '../service/vehicle_modeling_service_protocol.ts';
import { WrappableElementName } from './WrappableElementName.tsx';

// What a row's "+" that attaches bodies asks first: which of the bodies it
// lists, by name, in the order the row lists them.

export function BodyToAttachChooser(props: { bodies: readonly ElementReference[]; onChoose: (body: ElementReference) => void; onCancel: () => void }) {
  return (
    <div className="creation-form">
      <div className="property-list">
        <div className="chooser-choices">
          {props.bodies.map((body) => (
            <button type="button" key={body.name} className="chooser-choice element-choice" onClick={() => props.onChoose(body)}>
              <WrappableElementName name={body.name} />
            </button>
          ))}
        </div>
      </div>
      <div className="form-actions">
        <div className="form-buttons">
          <button type="button" className="chip" onClick={props.onCancel}>
            <span className="en">Cancel</span>
            <span className="zh">取消</span>
          </button>
        </div>
      </div>
    </div>
  );
}
