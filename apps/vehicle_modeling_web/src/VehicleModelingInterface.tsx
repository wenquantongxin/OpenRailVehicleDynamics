import { useCallback, useEffect, useMemo, useState, useSyncExternalStore } from 'react';

import { assignPartsToDrawnBodies } from './scene/vehicle_preview_scene.ts';
import { fetchProjectStartingPoints, type ProjectStartingPoint } from './service/fetch_project_starting_points.ts';
import { fetchVehicleProjectsDirectory } from './service/fetch_vehicle_projects_directory.ts';
import { modelingServerCommandChannel } from './service/modeling_server_command_channel.ts';
import { subjectElement, type AttachmentSetting, type ElementReference, type PropertySubject } from './service/vehicle_modeling_service_protocol.ts';
import { readStartLinkFromPageAddress } from './session/read_start_link_from_page_address.ts';
import { VehicleEditingSession, type EditRefusal, type EditValue, type OpenChoice, type StartLink } from './session/vehicle_editing_session.ts';
import { DiscardUnsavedChangesDialog } from './ui/DiscardUnsavedChangesDialog.tsx';

import { editedProjectInSentence, openChoiceInSentence } from './ui/open_choice_and_edited_project_wording.ts';
import { ProjectSaveAsDialog } from './ui/ProjectSaveAsDialog.tsx';
import { SelectionPropertyPanel } from './ui/SelectionPropertyPanel.tsx';
import { ServiceMessagesPanel } from './ui/ServiceMessagesPanel.tsx';
import type { PropertyFocusRequest } from './ui/SubjectPropertyList.tsx';

import { VehicleModelingStartPage } from './ui/VehicleModelingStartPage.tsx';
import { VehicleModelingTopBar } from './ui/VehicleModelingTopBar.tsx';
import { VehicleModelTree } from './ui/VehicleModelTree.tsx';
import { VehicleModelView } from './ui/VehicleModelView.tsx';

// The modelling interface: top bar, component tree, 3D view, property panel
// and message area around one editing session, which starts from what the
// service has open; the page's link (read_start_link_from_page_address.ts) is
// followed only when nothing is. A link is handled once: the page's address
// drops it, so that a reload resumes the session without it.

const serverOrigin = window.location.origin;

export function VehicleModelingInterface() {
  const session = useMemo(() => new VehicleEditingSession(modelingServerCommandChannel(serverOrigin)), []);
  const state = useSyncExternalStore(session.subscribe, () => session.state);
  const [startingPoints, setStartingPoints] = useState<ProjectStartingPoint[] | null>(null);
  const [startupProblem, setStartupProblem] = useState<string | null>(null);
  const [focusRequest, setFocusRequest] = useState<PropertyFocusRequest | null>(null);

  useEffect(() => {
    let superseded = false;
    const start = async (): Promise<void> => {
      const [listed, directory] = await Promise.all([fetchProjectStartingPoints(serverOrigin), fetchVehicleProjectsDirectory(serverOrigin)]);
      if (superseded) return;
      setStartingPoints(listed);
      let link: StartLink | null = null;
      try {
        link = readStartLinkFromPageAddress(new URLSearchParams(window.location.search), listed);
      } catch (error) {
        setStartupProblem(error instanceof Error ? error.message : String(error));
      }
      await session.start(directory, link);
      if (window.location.search !== '') window.history.replaceState(null, '', window.location.pathname);
    };
    start().catch((error: unknown) => {
      if (!superseded) setStartupProblem(error instanceof Error ? error.message : String(error));
    });
    return () => {
      superseded = true;
    };
  }, [session]);

  useEffect(() => {
    const onKey = (event: KeyboardEvent): void => {
      if (!(event.ctrlKey || event.metaKey) || event.altKey) return;
      const key = event.key.toLowerCase();
      const now = session.state;
      const asking = now.saveAs !== null || now.pendingOpen !== null;
      if (key === 's') {
        // Saving never reaches the browser, wherever the focus is.
        event.preventDefault();
        if (asking || now.vehicle === null) return;
        // A field commits what is typed in it when it loses the focus: that edit is then saved too.
        if (document.activeElement instanceof HTMLElement) document.activeElement.blur();
        void session.save();
        return;
      }
      const target = event.target as HTMLElement;
      if (target.tagName === 'INPUT' || asking) return;
      const redo = key === 'y' || (key === 'z' && event.shiftKey);
      if (key === 'z' && !event.shiftKey && now.canUndo) {
        event.preventDefault();
        void session.undo();
      } else if (redo && now.canRedo) {
        event.preventDefault();
        void session.redo();
      }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [session]);

  const partsWithoutBody = useMemo(
    () => (state.preview === null ? [] : assignPartsToDrawnBodies(state.vehicle?.visualDefinition ?? null, state.preview).withoutBody),
    [state.vehicle, state.preview],
  );
  const open = useCallback((choice: OpenChoice) => void session.open(choice), [session]);
  const select = useCallback((subject: PropertySubject | null) => void session.select(subject), [session]);
  const selectElement = useCallback((element: ElementReference | null) => void session.select(element === null ? null : { element }), [session]);
  const commit = useCallback((subject: PropertySubject, property: string, value: EditValue) => session.setProperty(subject, property, value), [session]);
  const create = useCallback(
    (kind: string, name: string, properties: Readonly<Record<string, EditValue>>, attachment: AttachmentSetting | null) => session.createElement(kind, name, properties, attachment),
    [session],
  );
  const showRefusal = useCallback(
    (refusal: EditRefusal) => {
      void session.select(refusal.subject);
      setFocusRequest((previous) => ({ subject: refusal.subject, property: refusal.property, serial: (previous?.serial ?? 0) + 1 }));
    },
    [session],
  );
  const showStartPage = state.choosingWhatToOpen || (state.vehicle === null && (state.started || startupProblem !== null));

  return (
    <div className={state.opening === null ? 'app' : 'app opening'}>
      <VehicleModelingTopBar
        state={state}
        onShowOpenChoices={() => void session.showOpenChoices()}
        onSave={() => void session.save()}
        onSaveAs={() => void session.startSaveAs()}
        onUndo={() => void session.undo()}
        onRedo={() => void session.redo()}
      />
      {/* While a vehicle opens, the panels still show the project it replaces and take no input. */}
      <div className="workspace" inert={state.opening !== null}>
        <VehicleModelTree
          tree={state.modelTree}
          elementCount={state.outline?.elements.length ?? null}
          kinds={state.elementKinds}
          openedVehicle={state.vehicle}
          selection={state.selection}
          creatingFromRow={state.creation?.rowKey ?? state.attachmentChange?.fromRow?.rowKey ?? null}
          onSelect={select}
          onCreate={(rowKey, offers) => void session.startCreation(rowKey, offers)}
          onAttach={(rowKey, bodies) => void session.startAttaching(rowKey, bodies)}
        />
        <VehicleModelView
          vehicle={state.vehicle}
          preview={state.preview}
          revision={state.revision}
          selection={subjectElement(state.selection)}
          checkRefusals={state.check?.report.refusals ?? []}
          kinds={state.elementKinds}
          onPick={selectElement}
        >
          {showStartPage && (
            <VehicleModelingStartPage
              startingPoints={startingPoints}
              vehicleProjects={state.vehicleProjects}
              vehicleProjectsDirectory={state.vehicleProjectsDirectory}
              problem={startupProblem ?? state.commandRefusal?.message ?? null}
              shownOver={state.vehicle === null ? null : (state.vehicleProjectFile?.name ?? state.outline?.vehicleName ?? '')}
              onOpen={open}
              onLeave={() => session.leaveOpenChoices()}
            />
          )}
        </VehicleModelView>
        <SelectionPropertyPanel
          state={state}
          focusRequest={focusRequest}
          onCommit={commit}
          onDismissRefusal={() => session.dismissEditRefusal()}
          onSelectElement={selectElement}
          onChooseCreationOffer={(offer) => void session.chooseCreationOffer(offer)}
          onCreate={create}
          onCancelCreation={() => void session.cancelCreation()}
          onStartChangingAttachment={() => void session.startChangingAttachment()}
          onChooseBodyToAttach={(body) => void session.chooseBodyToAttach(body)}
          onSetAttachment={(setting) => session.setAttachment(setting)}
          onCancelAttachmentChange={() => void session.cancelAttachmentChange()}
          onRequestRemoval={(element) => void session.requestRemoval(element)}
          onConfirmRemoval={() => void session.confirmRemoval()}
          onCancelRemoval={() => void session.cancelRemoval()}
        />
        <ServiceMessagesPanel state={state} partsWithoutBody={partsWithoutBody} onSelect={selectElement} onShowRefusal={showRefusal} />
      </div>
      {state.saveAs !== null && state.vehicleProjectsDirectory !== null && (
        <ProjectSaveAsDialog
          dialog={state.saveAs}
          vehicleProjectsDirectory={state.vehicleProjectsDirectory}
          onSave={(name, replacingAnotherProject) => void session.saveAs(name, replacingAnotherProject)}
          onChooseAnotherName={() => session.chooseAnotherName()}
          onCancel={() => session.cancelSaveAs()}
        />
      )}
      {state.pendingOpen !== null && (
        <DiscardUnsavedChangesDialog
          edited={editedProjectInSentence(state)}
          chosen={openChoiceInSentence(state.pendingOpen)}
          onDiscard={() => void session.discardChangesAndOpen()}
          onKeepEditing={() => session.keepEditing()}
        />
      )}
    </div>
  );
}
