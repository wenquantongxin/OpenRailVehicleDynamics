import type { PropertySubject } from '../service/vehicle_modeling_service_protocol.ts';
import type { EditRefusal, SetProperty } from '../session/vehicle_editing_session.ts';

/** What every editor of a choice, a reference or a token, is given about the property it commits. */
export interface ChoicePropertyEditorProps {
  subject: PropertySubject;
  refusal: EditRefusal | null;
  /** Takes no choice: the element's rename is on its way. */
  frozen: boolean;
  onCommit: SetProperty;
}
