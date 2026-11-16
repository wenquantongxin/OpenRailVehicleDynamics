import { sameElement, type ElementReference } from '../service/vehicle_modeling_service_protocol.ts';
import type { SessionState, SetProperty } from '../session/vehicle_editing_session.ts';
import { DockedPanelTitleStrip } from './DockedPanelTitleStrip.tsx';
import { ElementCreationForm, type CreateElement } from './ElementCreationForm.tsx';
import { ElementCreationHead } from './ElementCreationHead.tsx';
import { ElementPropertyList, type PropertyFocusRequest } from './ElementPropertyList.tsx';
import { ElementRemovalConfirmation } from './ElementRemovalConfirmation.tsx';
import { SelectedElementHead } from './SelectedElementHead.tsx';

// The property panel: the selected element's properties, which it can also
// remove; the question a removal asks when it takes more than the element; or
// the form that creates an element of a kind.

export function ElementPropertyPanel(props: {
  state: SessionState;
  focusRequest: PropertyFocusRequest | null;
  onCommit: SetProperty;
  onDismissRefusal: () => void;
  onCreate: CreateElement;
  onCancelCreation: () => void;
  onRequestRemoval: (element: ElementReference) => void;
  onConfirmRemoval: () => void;
  onCancelRemoval: () => void;
}) {
  const { state } = props;
  const { selection } = state;
  const elements = state.outline?.elements ?? [];
  const creating = state.elementKinds.find((kind) => kind.kind === state.creatingKind);
  return (
    <aside className="panel properties-panel">
      <DockedPanelTitleStrip en="Properties" zh="属性" detail={selection === null ? undefined : `${state.properties.length}`} />
      {creating !== undefined ? (
        <>
          <ElementCreationHead kind={creating.kind} />
          <ElementCreationForm key={creating.kind} kind={creating} elements={elements} onCreate={props.onCreate} onCancel={props.onCancelCreation} />
        </>
      ) : selection === null ? (
        <p className="panel-empty">
          Select an element in the tree or the 3D view. <span className="zh">在组件树或三维视图中选择元素。</span>
        </p>
      ) : (
        <>
          <SelectedElementHead
            selection={selection}
            removalPending={state.pendingRemoval !== null}
            refusal={state.commandRefusal !== null && sameElement(state.commandRefusal.element, selection) ? state.commandRefusal : null}
            onRemove={() => props.onRequestRemoval(selection)}
          />
          {state.pendingRemoval !== null ? (
            <ElementRemovalConfirmation removal={state.pendingRemoval} onConfirm={props.onConfirmRemoval} onCancel={props.onCancelRemoval} />
          ) : (
            <ElementPropertyList
              selection={selection}
              properties={state.properties}
              elements={elements}
              editRefusal={state.editRefusal}
              renaming={state.renaming}
              focusRequest={props.focusRequest}
              onCommit={props.onCommit}
              onDismissRefusal={props.onDismissRefusal}
            />
          )}
        </>
      )}
    </aside>
  );
}
