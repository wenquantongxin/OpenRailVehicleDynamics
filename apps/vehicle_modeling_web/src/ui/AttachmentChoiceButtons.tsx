import type { AttachmentDescription, ElementKindDescription } from '../service/vehicle_modeling_service_protocol.ts';
import { sameAttachmentChoice, type AttachmentChoice } from './body_attachment_form_rules.ts';
import { BilingualTitleText } from './BilingualTitleText.tsx';
import { elementKindTitle } from './find_element_kind_title.ts';

// The choice of how a body is to be attached: free, under the title the
// service gives it, and each joint kind by its title; one of them at a time.

export function AttachmentChoiceButtons(props: {
  description: AttachmentDescription;
  kinds: readonly ElementKindDescription[];
  choices: readonly AttachmentChoice[];
  chosen: AttachmentChoice | null;
  disabled: boolean;
  onChoose: (choice: AttachmentChoice) => void;
}) {
  return (
    <div className="attachment-choices" role="radiogroup" aria-label={props.description.title.en}>
      {props.choices.map((choice) => (
        <button
          type="button"
          key={'free' in choice ? '' : choice.jointKind}
          className="attachment-choice"
          role="radio"
          aria-checked={sameAttachmentChoice(props.chosen, choice)}
          disabled={props.disabled}
          onClick={() => props.onChoose(choice)}
        >
          <BilingualTitleText title={'free' in choice ? props.description.freeTitle : elementKindTitle(props.kinds, choice.jointKind)} />
        </button>
      ))}
    </div>
  );
}
