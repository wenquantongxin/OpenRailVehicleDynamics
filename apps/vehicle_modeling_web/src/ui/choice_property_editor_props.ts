import type { ElementReference } from '../service/vehicle_modeling_service_protocol.ts';
import type { EditRefusal, SetProperty } from '../session/vehicle_editing_session.ts';

/** What every editor of a choice, a reference, a token or a boolean, is given about the property it commits. */
export interface ChoicePropertyEditorProps {
  element: ElementReference;
  refusal: EditRefusal | null;
  /** Takes no choice: the element's rename is on its way. */
  frozen: boolean;
  onCommit: SetProperty;
}
