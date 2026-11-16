import { useCallback, useEffect, useMemo, useState, useSyncExternalStore } from 'react';

import { assignPartsToDrawnBodies } from './scene/preview_scene.ts';
import { commandChannel } from './service/command_channel.ts';
import type { ElementReference } from './service/protocol.ts';
import { fetchReferenceVehicles, fetchVisualDefinitionText, type ReferenceVehicle } from './service/reference_vehicles.ts';
import { EditingSession, type EditRefusal, type EditValue } from './session/editing_session.ts';
import { ComponentTree } from './ui/ComponentTree.tsx';
import { MessageArea } from './ui/MessageArea.tsx';
import { ModelView } from './ui/ModelView.tsx';
import { PanelHead } from './ui/PanelHead.tsx';
import { PropertyPanel, type FocusRequest } from './ui/PropertyPanel.tsx';
import { TopBar } from './ui/TopBar.tsx';
import { parseVisualDefinition, type VisualDefinition } from './visual/visual_definition.ts';

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
        <PanelHead en="Open a vehicle" zh="打开车辆" />
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

export function App() {
  const session = useMemo(() => new EditingSession(commandChannel(serverOrigin), loadVisualDefinition), []);
  const state = useSyncExternalStore(session.subscribe, () => session.state);
  const [vehicles, setVehicles] = useState<ReferenceVehicle[] | null>(null);
  const [startupProblem, setStartupProblem] = useState<string | null>(null);
  const [focusRequest, setFocusRequest] = useState<FocusRequest | null>(null);

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
  const showRefusal = useCallback(
    (refusal: EditRefusal) => {
      void session.select(refusal.element);
      setFocusRequest((previous) => ({ element: refusal.element, property: refusal.property, serial: (previous?.serial ?? 0) + 1 }));
    },
    [session],
  );

  return (
    <div className={state.opening === null ? 'app' : 'app opening'}>
      <TopBar vehicles={vehicles} state={state} onOpen={open} onUndo={() => void session.undo()} onRedo={() => void session.redo()} />
      {/* While a vehicle opens, the panels still show the project it replaces and take no input. */}
      <div className="workspace" inert={state.opening !== null}>
        <ComponentTree outline={state.outline} selection={state.selection} onSelect={select} />
        <ModelView
          vehicle={state.vehicle}
          preview={state.preview}
          revision={state.revision}
          selection={state.selection}
          checkRefusals={state.check?.report.refusals ?? []}
          onPick={select}
        >
          {state.vehicle === null && <OpenStage vehicles={vehicles} problem={startupProblem} onOpen={open} />}
        </ModelView>
        <PropertyPanel
          selection={state.selection}
          properties={state.properties}
          editRefusal={state.editRefusal}
          focusRequest={focusRequest}
          onCommit={commit}
          onDismissRefusal={() => session.dismissEditRefusal()}
        />
        <MessageArea state={state} partsWithoutBody={partsWithoutBody} onSelect={select} onShowRefusal={showRefusal} />
      </div>
    </div>
  );
}
