import { useEffect, useRef, useState, type ReactNode } from 'react';

import { framesCamera, viewPresets, type ViewPreset } from '../scene/vehicle_model_view_camera_presets.ts';
import { VehicleModelViewport } from '../scene/vehicle_model_viewport.ts';
import { drawsAnything, type PartsDisplay, type SceneDisplay } from '../scene/vehicle_preview_scene.ts';
import type { ElementKindDescription, ElementReference, Preview } from '../service/vehicle_modeling_service_protocol.ts';
import type { OpenedVehicle } from '../session/vehicle_editing_session.ts';
import { elementKindTitle } from './find_element_kind_title.ts';

// The 3D view with its controls: view presets, how body parts are shown, the
// fixed frames switch, a legend, and a badge when the drawing is older than
// the project. The camera frames the vehicle when it is opened and is left
// alone after edits. Without WebGL the view says so, and the panels around it
// work as before.

const presetNames: Record<ViewPreset, { en: string; zh: string }> = {
  overview: { en: 'Overview', zh: '整车' },
  side: { en: 'Side', zh: '侧视' },
  plan: { en: 'Plan', zh: '俯视' },
  front: { en: 'Front', zh: '正面' },
};

const partsModes: { mode: PartsDisplay; en: string; zh: string }[] = [
  { mode: 'solid', en: 'Solid', zh: '实体' },
  { mode: 'xray', en: 'X-ray', zh: '透视' },
  { mode: 'hidden', en: 'Hidden', zh: '隐藏' },
];

export function VehicleModelView(props: {
  vehicle: OpenedVehicle | null;
  preview: Preview | null;
  revision: number;
  selection: ElementReference | null;
  /** The latest check's refusals, shown while there is no drawing. */
  checkRefusals: string[];
  /** The element kinds with their titles, which name a drawing under the pointer. */
  kinds: readonly ElementKindDescription[];
  onPick: (element: ElementReference | null) => void;
  /** Shown over the view, such as the choice of what to open. */
  children?: ReactNode;
}) {
  const { vehicle, preview } = props;
  const container = useRef<HTMLDivElement>(null);
  const viewport = useRef<VehicleModelViewport | null>(null);
  const fittedVehicle = useRef<OpenedVehicle | null>(null);
  const onPick = useRef(props.onPick);
  onPick.current = props.onPick;
  const kinds = useRef(props.kinds);
  kinds.current = props.kinds;
  const [display, setDisplay] = useState<SceneDisplay>({ parts: 'solid', frames: false });
  const [viewFailure, setViewFailure] = useState<string | null>(null);
  const [drawingFailure, setDrawingFailure] = useState<string | null>(null);

  useEffect(() => {
    let created: VehicleModelViewport;
    try {
      created = new VehicleModelViewport(container.current!, {
        // A click on empty space clears the selection.
        pick: (drawing) => onPick.current(drawing === null ? null : drawing.element),
        describe: (drawing) => `${elementKindTitle(kinds.current, drawing.element.kind).en} · ${drawing.element.name}`,
      });
    } catch (error) {
      setViewFailure(error instanceof Error ? error.message : String(error));
      return;
    }
    viewport.current = created;
    return () => {
      created.dispose();
      viewport.current = null;
      fittedVehicle.current = null;
    };
  }, []);

  useEffect(() => {
    const view = viewport.current;
    if (view === null) return;
    const fit = framesCamera(preview, fittedVehicle.current === vehicle);
    try {
      view.showPreview(preview, vehicle?.visualDefinition ?? null, fit);
      setDrawingFailure(null);
      if (fit) fittedVehicle.current = vehicle;
    } catch (error) {
      view.showPreview(null, null, false);
      setDrawingFailure(error instanceof Error ? error.message : String(error));
    }
  }, [vehicle, preview]);

  useEffect(() => {
    viewport.current?.setHighlight(props.selection);
  }, [props.selection]);

  useEffect(() => {
    viewport.current?.setDisplay(display);
  }, [display]);

  const stale = preview !== null && preview.projectRevision < props.revision;
  return (
    <main className="view-area">
      <div className="viewport" ref={container} />
      {viewFailure !== null && (
        <div className="view-notice failure" role="alert">
          The 3D view cannot start. The tree, the properties and the messages work without it.
          <span className="zh">三维视图无法启动；组件树、属性与消息不受影响。</span>
          <span className="detail">{viewFailure}</span>
        </div>
      )}
      {viewFailure === null && preview !== null && drawsAnything(preview) && (
        <>
          <div className="view-toolbar left">
            <span className="toolbar-label">
              View <span className="zh">视角</span>
            </span>
            {viewPresets.map((preset) => (
              <button type="button" key={preset} className="chip" onClick={() => viewport.current?.applyPreset(preset)}>
                <span className="en">{presetNames[preset].en}</span>
                <span className="zh">{presetNames[preset].zh}</span>
              </button>
            ))}
          </div>
          <div className="view-toolbar right">
            <div className="segmented" role="radiogroup" aria-label="Body parts">
              <span className="segmented-label">
                <span className="en">Parts</span>
                <span className="zh">部件</span>
              </span>
              {partsModes.map((entry) => (
                <button
                  type="button"
                  key={entry.mode}
                  role="radio"
                  aria-checked={display.parts === entry.mode}
                  className={display.parts === entry.mode ? 'on' : ''}
                  onClick={() => setDisplay({ ...display, parts: entry.mode })}
                >
                  <span className="en">{entry.en}</span>
                  <span className="zh">{entry.zh}</span>
                </button>
              ))}
            </div>
            <button type="button" className={`chip ${display.frames ? 'on' : ''}`} aria-pressed={display.frames} onClick={() => setDisplay({ ...display, frames: !display.frames })}>
              <span className="en">Frames</span>
              <span className="zh">坐标系</span>
            </button>
          </div>
          {stale && (
            <div className="stale-badge" role="status">
              <span className="en">Stale</span>
              <span className="zh">预览已过期</span>
              <span className="detail">
                drawing of revision {preview.projectRevision} · project at revision {props.revision}
              </span>
            </div>
          )}
          <div className="legend" aria-hidden="true">
            <span>
              <i className="swatch force" />
              Force element <span className="zh">力元连线</span>
            </span>
            <span>
              <i className="swatch joint" />
              Joint mark <span className="zh">铰点</span>
            </span>
            <span>
              <i className="swatch body" />
              Body without parts <span className="zh">无外形刚体</span>
            </span>
            <span>
              <i className="swatch selected" />
              Selected <span className="zh">选中</span>
            </span>
          </div>
        </>
      )}
      {viewFailure === null && vehicle !== null && preview === null && (
        <div className="view-notice no-drawing">
          No drawing until the check passes: since it was opened, the check has refused this project.
          <span className="zh">检查通过之前没有图形：本工程自打开以来未通过检查。</span>
          {props.checkRefusals.length > 0 && (
            <ul className="notice-refusals">
              {props.checkRefusals.map((refusal, index) => (
                <li key={index}>{refusal}</li>
              ))}
            </ul>
          )}
        </div>
      )}
      {viewFailure === null && preview !== null && !drawsAnything(preview) && (
        <div className="view-notice">
          Nothing can be drawn: the preview lists every element as not drawable. A free body without a placement is not drawn, nor is anything
          jointed to it or mounted on it; the list is in the messages below.
          <span className="zh">无可绘制内容：预览将所有元素列为无法绘制。没有放置位置的自由刚体及与之相连的一切均无法绘制，清单见下方消息。</span>
        </div>
      )}
      {viewFailure === null && drawingFailure !== null && (
        <div className="view-notice failure" role="alert">
          The project cannot be drawn: {drawingFailure} <span className="zh">无法绘制该工程</span>
        </div>
      )}
      {props.children}
    </main>
  );
}
