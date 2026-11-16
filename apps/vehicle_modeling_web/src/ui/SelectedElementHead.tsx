import type { ElementReference, Refusal } from '../service/vehicle_modeling_service_protocol.ts';
import { identifierLabel } from './identifier_display_labels.ts';
import { WrappableElementName } from './WrappableElementName.tsx';

// The head of the property panel for the selected element: its kind, its
// name, the control that removes it, and the service's reason when it
// refused to remove it.

export function SelectedElementHead(props: { selection: ElementReference; removalPending: boolean; refusal: Refusal | null; onRemove: () => void }) {
  return (
    <div className="selection-head">
      <div className="selection-title">
        <span className="selection-kind">{identifierLabel(props.selection.kind)}</span>
        <span className="selection-name">
          <WrappableElementName name={props.selection.name} />
        </span>
      </div>
      <button type="button" className="chip remove-button" disabled={props.removalPending} onClick={props.onRemove}>
        <span className="en">Remove</span>
        <span className="zh">删除</span>
      </button>
      {props.refusal !== null && (
        <p className="field-message refused head-refusal" role="alert">
          {props.refusal.message}
        </p>
      )}
    </div>
  );
}
