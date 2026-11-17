import { rowPathOfSubject, rowWithKey } from '../service/find_model_tree_rows.ts';
import { sameSubject, subjectElement, type CreationOffer, type ElementReference } from '../service/vehicle_modeling_service_protocol.ts';
import type { SessionState, SetProperty } from '../session/vehicle_editing_session.ts';
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
// removes it, and the question a removal asks. For selected settings: their
// properties under the row's title, which are neither removed nor renamed.
// For a row's "+": the choice of a kind when it offers several, then the form
// that creates one.

export function SelectionPropertyPanel(props: {
  state: SessionState;
  focusRequest: PropertyFocusRequest | null;
  onCommit: SetProperty;
  onDismissRefusal: () => void;
  onChooseCreationOffer: (offer: CreationOffer) => void;
  onCreate: CreateElement;
  onCancelCreation: () => void;
  onRequestRemoval: (element: ElementReference) => void;
  onConfirmRemoval: () => void;
  onCancelRemoval: () => void;
}) {
  const { state } = props;
  const { selection, creation } = state;
  const elements = state.outline?.elements ?? [];
  const tree = state.modelTree ?? [];
  const selectedElement = subjectElement(selection);
  const selectedPath = selection === null ? null : rowPathOfSubject(tree, selection);
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
  } else if (selection === null) {
    shown = (
      <p className="panel-empty">
        Select a row of the model tree, or an element in the 3D view. <span className="zh">在模型树中选择一行，或在三维视图中选择元素。</span>
      </p>
    );
  } else if (selectedElement !== null) {
    shown = (
      <>
        <SelectedElementHead
          selection={selectedElement}
          kindTitle={elementKindTitle(state.elementKinds, selectedElement.kind)}
          removalPending={state.pendingRemoval !== null}
          refusal={state.commandRefusal !== null && sameSubject(state.commandRefusal.subject, selection) ? state.commandRefusal : null}
          onRemove={() => props.onRequestRemoval(selectedElement)}
        />
        {state.pendingRemoval !== null ? (
          <ElementRemovalConfirmation removal={state.pendingRemoval} kinds={state.elementKinds} onConfirm={props.onConfirmRemoval} onCancel={props.onCancelRemoval} />
        ) : (
          properties
        )}
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
      <DockedPanelTitleStrip en="Properties" zh="属性" detail={selection === null ? undefined : `${state.properties.length}`} />
      {shown}
    </aside>
  );
}
