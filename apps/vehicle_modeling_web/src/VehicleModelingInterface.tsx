import { useCallback, useEffect, useMemo, useState, useSyncExternalStore } from 'react';

import { assignPartsToDrawnBodies } from './scene/vehicle_preview_scene.ts';
import { fetchReferenceVehicles, fetchVisualDefinitionText, type ReferenceVehicle } from './service/fetch_reference_vehicles.ts';
import { modelingServerCommandChannel } from './service/modeling_server_command_channel.ts';
import type { ElementReference } from './service/vehicle_modeling_service_protocol.ts';
import { VehicleEditingSession, type EditRefusal, type EditValue } from './session/vehicle_editing_session.ts';
import { DockedPanelTitleStrip } from './ui/DockedPanelTitleStrip.tsx';
import type { PropertyFocusRequest } from './ui/ElementPropertyList.tsx';
import { ElementPropertyPanel } from './ui/ElementPropertyPanel.tsx';
import { ServiceMessagesPanel } from './ui/ServiceMessagesPanel.tsx';
import { VehicleElementTree } from './ui/VehicleElementTree.tsx';
import { VehicleModelingTopBar } from './ui/VehicleModelingTopBar.tsx';
import { VehicleModelView } from './ui/VehicleModelView.tsx';
import { parseVisualDefinition, type VisualDefinition } from './visual/read_vehicle_visual_definition.ts';

// The modelling interface: top bar, component tree, 3D view, property panel
// and message area around one editing session. Deep links: ?vehicle=<name>
// opens that reference vehicle, &select=<kind>:<name> then selects an element.

const query = new URLSearchParams(window.location.search);
const serverOrigin = window.location.origin;

async function loadVisualDefinition(vehicle: ReferenceVehicle): Promise<VisualDefinition | null> {
  return vehicle.visualDefinitionAvailable ? parseVisualDefinition(await fetchVisualDefinitionText(serverOrigin, vehicle)) : null;
}

/** The element of a `select` link; the kind runs up to the first colon. */
function linkedElement(link: string): ElementReference {
  const separator = link.indexOf(':');
  if (separator < 1) {
    throw new Error(`the select link '${link}' is not <kind>:<name>`);
  }
  return { kind: link.slice(0, separator), name: link.slice(separator + 1) };
}

function OpenStage(props: { vehicles: ReferenceVehicle[] | null; problem: string | null; onOpen: (vehicle: ReferenceVehicle) => void }) {
  return (
    <div className="open-stage">
      <div className="open-card">
        <DockedPanelTitleStrip en="Open a vehicle" zh="打开车辆" />
        <p>
          Choose one of the reference vehicles the modelling server was started with.
          <span className="zh">选择建模服务器启动时给出的参考车辆。</span>
        </p>
        {props.problem !== null && <p className="open-problem">{props.problem}</p>}
        <div className="open-choices">
          {(props.vehicles ?? []).map((vehicle) => (
            <button type="button" key={vehicle.name} className="open-choice" onClick={() => props.onOpen(vehicle)}>
              <span className="en">{vehicle.title.en}</span>
              <span className="zh">{vehicle.title.zh}</span>
            </button>
          ))}
        </div>
      </div>
    </div>
  );
}

export function VehicleModelingInterface() {
  const session = useMemo(() => new VehicleEditingSession(modelingServerCommandChannel(serverOrigin), loadVisualDefinition), []);
  const state = useSyncExternalStore(session.subscribe, () => session.state);
  const [vehicles, setVehicles] = useState<ReferenceVehicle[] | null>(null);
  const [startupProblem, setStartupProblem] = useState<string | null>(null);
  const [focusRequest, setFocusRequest] = useState<PropertyFocusRequest | null>(null);

  useEffect(() => {
    let superseded = false;
    const start = async (): Promise<void> => {
      const listed = await fetchReferenceVehicles(serverOrigin);
      if (superseded) return;
      setVehicles(listed);
      const name = query.get('vehicle');
      if (name === null) return;
      const vehicle = listed.find((entry) => entry.name === name);
      if (vehicle === undefined) {
        throw new Error(`no reference vehicle is named '${name}'`);
      }
      const link = query.get('select');
      const element = link === null ? null : linkedElement(link);
      await session.open(vehicle);
      if (element !== null) await session.select(element);
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
      const target = event.target as HTMLElement;
      if (target.tagName === 'INPUT' || !(event.ctrlKey || event.metaKey) || event.altKey) return;
      const key = event.key.toLowerCase();
      const redo = key === 'y' || (key === 'z' && event.shiftKey);
      if (key === 'z' && !event.shiftKey && session.state.canUndo) {
        event.preventDefault();
        void session.undo();
      } else if (redo && session.state.canRedo) {
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
  const open = useCallback((vehicle: ReferenceVehicle) => void session.open(vehicle), [session]);
  const select = useCallback((element: ElementReference | null) => void session.select(element), [session]);
  const commit = useCallback((element: ElementReference, property: string, value: EditValue) => session.setProperty(element, property, value), [session]);
  const create = useCallback(
    (kind: string, name: string, properties: Readonly<Record<string, EditValue>>) => session.createElement(kind, name, properties),
    [session],
  );
  const elementKinds = useMemo(() => state.elementKinds.map((kind) => kind.kind), [state.elementKinds]);
  const showRefusal = useCallback(
    (refusal: EditRefusal) => {
      void session.select(refusal.element);
      setFocusRequest((previous) => ({ element: refusal.element, property: refusal.property, serial: (previous?.serial ?? 0) + 1 }));
    },
    [session],
  );

  return (
    <div className={state.opening === null ? 'app' : 'app opening'}>
      <VehicleModelingTopBar vehicles={vehicles} state={state} onOpen={open} onUndo={() => void session.undo()} onRedo={() => void session.redo()} />
      {/* While a vehicle opens, the panels still show the project it replaces and take no input. */}
      <div className="workspace" inert={state.opening !== null}>
        <VehicleElementTree
          kinds={elementKinds}
          outline={state.outline}
          selection={state.selection}
          creatingKind={state.creatingKind}
          onSelect={select}
          onCreate={(kind) => void session.startCreation(kind)}
        />
        <VehicleModelView
          vehicle={state.vehicle}
          preview={state.preview}
          revision={state.revision}
          selection={state.selection}
          checkRefusals={state.check?.report.refusals ?? []}
          onPick={select}
        >
          {state.vehicle === null && <OpenStage vehicles={vehicles} problem={startupProblem} onOpen={open} />}
        </VehicleModelView>
        <ElementPropertyPanel
          state={state}
          focusRequest={focusRequest}
          onCommit={commit}
          onDismissRefusal={() => session.dismissEditRefusal()}
          onCreate={create}
          onCancelCreation={() => void session.cancelCreation()}
          onRequestRemoval={(element) => void session.requestRemoval(element)}
          onConfirmRemoval={() => void session.confirmRemoval()}
          onCancelRemoval={() => void session.cancelRemoval()}
        />
        <ServiceMessagesPanel state={state} partsWithoutBody={partsWithoutBody} onSelect={select} onShowRefusal={showRefusal} />
      </div>
    </div>
  );
}
