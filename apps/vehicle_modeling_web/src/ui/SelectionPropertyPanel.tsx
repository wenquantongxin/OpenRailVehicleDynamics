import { rowPathOfSubject, rowWithKey } from '../service/find_model_tree_rows.ts';
import { sameSubject, subjectElement, type AttachmentSetting, type CreationOffer, type ElementReference } from '../service/vehicle_modeling_service_protocol.ts';
import type { SessionState, SetProperty } from '../session/vehicle_editing_session.ts';
import { AttachmentChangeForm } from './AttachmentChangeForm.tsx';
import { BodyAttachingHead } from './BodyAttachingHead.tsx';
import { BodyAttachmentSummary } from './BodyAttachmentSummary.tsx';
import { BodyToAttachChooser } from './BodyToAttachChooser.tsx';
import { CreationOfferChooser } from './CreationOfferChooser.tsx';
import { DockedPanelTitleStrip } from './DockedPanelTitleStrip.tsx';
import { ElementCreationForm, type CreateElement } from './ElementCreationForm.tsx';
import { ElementCreationHead } from './ElementCreationHead.tsx';
import { ElementRemovalConfirmation } from './ElementRemovalConfirmation.tsx';
import { elementKindTitle } from './find_element_kind_title.ts';
import { SelectedElementHead } from './SelectedElementHead.tsx';
import { SelectedSettingsHead } from './SelectedSettingsHead.tsx';
import { SubjectPropertyList, type PropertyFocusRequest } from './SubjectPropertyList.tsx';

// The property panel. For a selected element: its properties, the control that
// removes it, and the question a removal asks; for an element that is
// attached, how it is attached comes first, and changing it takes the place of
// the properties while it is asked for. For selected settings: their
// properties under the row's title, which are neither removed nor renamed.
// For a row's "+": the choice of a kind when it offers several, then the form
// that creates one; or, for a row that attaches bodies, the choice of the
// body, then how it is to be attached.

export function SelectionPropertyPanel(props: {
  state: SessionState;
  focusRequest: PropertyFocusRequest | null;
  onCommit: SetProperty;
  onDismissRefusal: () => void;
  onSelectElement: (element: ElementReference) => void;
  onChooseCreationOffer: (offer: CreationOffer) => void;
  onCreate: CreateElement;
  onCancelCreation: () => void;
  onStartChangingAttachment: () => void;
  onChooseBodyToAttach: (body: ElementReference) => void;
  onSetAttachment: (setting: AttachmentSetting) => Promise<boolean>;
  onCancelAttachmentChange: () => void;
  onRequestRemoval: (element: ElementReference) => void;
  onConfirmRemoval: () => void;
  onCancelRemoval: () => void;
}) {
  const { state } = props;
  const { selection, creation, attachmentChange } = state;
  const elements = state.outline?.elements ?? [];
  const tree = state.modelTree ?? [];
  const selectedElement = subjectElement(selection);
  const selectedPath = selection === null ? null : rowPathOfSubject(tree, selection);
  const attachmentOf = (kind: string) => state.elementKinds.find((each) => each.kind === kind)?.attachment ?? null;
  const properties =
    selection === null ? null : (
      <SubjectPropertyList
        selection={selection}
        properties={state.properties}
        elements={elements}
        kinds={state.elementKinds}
        editRefusal={state.editRefusal}
        renaming={state.renaming}
        focusRequest={props.focusRequest}
        onCommit={props.onCommit}
        onDismissRefusal={props.onDismissRefusal}
      />
    );
  const changeForm = (description: NonNullable<ReturnType<typeof attachmentOf>>, offersFree: boolean) =>
    attachmentChange?.body != null && (
      <AttachmentChangeForm
        key={attachmentChange.body.element.name}
        description={description}
        kinds={state.elementKinds}
        body={attachmentChange.body}
        offersFree={offersFree}
        elements={elements}
        refusal={attachmentChange.refusal}
        onSelectElement={props.onSelectElement}
        onApply={props.onSetAttachment}
        onCancel={props.onCancelAttachmentChange}
      />
    );

  let shown;
  if (creation !== null) {
    const fromRow = rowWithKey(tree, creation.rowKey);
    const chosen = creation.chosen;
    shown = (
      <>
        <ElementCreationHead fromRow={fromRow} kindTitle={chosen === null ? null : elementKindTitle(state.elementKinds, chosen.kind)} />
        {chosen === null ? (
          <CreationOfferChooser kinds={state.elementKinds} offers={creation.offers} onChoose={props.onChooseCreationOffer} onCancel={props.onCancelCreation} />
        ) : (
          <ElementCreationForm
            key={`${creation.rowKey}/${chosen.kind}`}
            kind={state.elementKinds.find((kind) => kind.kind === chosen.kind)!}
            fixedProperties={chosen.fixedProperties}
            elements={elements}
            kinds={state.elementKinds}
            onCreate={props.onCreate}
            onCancel={props.onCancelCreation}
          />
        )}
      </>
    );
  } else if (attachmentChange !== null && attachmentChange.fromRow !== null) {
    const fromRow = attachmentChange.fromRow;
    const description = attachmentOf(attachmentChange.body?.element.kind ?? fromRow.bodies[0]!.kind)!;
    shown = (
      <>
        <BodyAttachingHead description={description} fromRow={rowWithKey(tree, fromRow.rowKey)} body={attachmentChange.body?.element ?? null} />
        {attachmentChange.body === null ? (
          <BodyToAttachChooser bodies={fromRow.bodies} onChoose={props.onChooseBodyToAttach} onCancel={props.onCancelAttachmentChange} />
        ) : (
          changeForm(description, false)
        )}
      </>
    );
  } else if (selection === null) {
    shown = (
      <p className="panel-empty">
        Select a row of the model tree, or an element in the 3D view. <span className="zh">在模型树中选择一行，或在三维视图中选择元素。</span>
      </p>
    );
  } else if (selectedElement !== null) {
    const description = attachmentOf(selectedElement.kind);
    let below;
    if (state.pendingRemoval !== null) {
      below = <ElementRemovalConfirmation removal={state.pendingRemoval} kinds={state.elementKinds} onConfirm={props.onConfirmRemoval} onCancel={props.onCancelRemoval} />;
    } else if (description !== null && attachmentChange?.body != null) {
      below = changeForm(description, true);
    } else {
      below = (
        <>
          {description !== null && state.attachment !== null && (
            <BodyAttachmentSummary
              description={description}
              attachment={state.attachment}
              kinds={state.elementKinds}
              onSelectElement={props.onSelectElement}
              onChange={props.onStartChangingAttachment}
            />
          )}
          {properties}
        </>
      );
    }
    shown = (
      <>
        <SelectedElementHead
          selection={selectedElement}
          kindTitle={elementKindTitle(state.elementKinds, selectedElement.kind)}
          removalPending={state.pendingRemoval !== null}
          refusal={state.commandRefusal !== null && sameSubject(state.commandRefusal.subject, selection) ? state.commandRefusal : null}
          onRemove={() => props.onRequestRemoval(selectedElement)}
        />
        {below}
      </>
    );
  } else {
    const row = selectedPath?.at(-1);
    shown = (
      <>
        {row?.title != null && <SelectedSettingsHead title={row.title} within={selectedPath !== null && selectedPath.length > 1 ? selectedPath[0]!.title : null} />}
        {properties}
      </>
    );
  }

  return (
    <aside className="panel properties-panel">
      <DockedPanelTitleStrip en="Properties" zh="属性" detail={selection === null || attachmentChange?.fromRow != null ? undefined : `${state.properties.length}`} />
      {shown}
    </aside>
  );
}
