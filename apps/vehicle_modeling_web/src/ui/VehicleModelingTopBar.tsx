import { useEffect, useRef, useState } from 'react';

import type { ReferenceVehicle } from '../service/fetch_reference_vehicles.ts';
import { checkStatus, type SessionState } from '../session/vehicle_editing_session.ts';

// Title, the vehicle and how to open one, undo and redo, and the check status
// with the project revision.

const checkStatusNames = {
  passed: { en: 'Check passed', zh: '检查通过' },
  failed: { en: 'Check failed', zh: '检查未通过' },
  not_checked: { en: 'Not checked', zh: '未检查' },
};

function OpenMenu(props: { vehicles: ReferenceVehicle[] | null; current: ReferenceVehicle | null; onOpen: (vehicle: ReferenceVehicle) => void }) {
  const [open, setOpen] = useState(false);
  const menu = useRef<HTMLDivElement>(null);
  useEffect(() => {
    if (!open) return;
    const close = (event: Event): void => {
      if (event instanceof KeyboardEvent ? event.key === 'Escape' : !menu.current?.contains(event.target as Node)) setOpen(false);
    };
    window.addEventListener('pointerdown', close);
    window.addEventListener('keydown', close);
    return () => {
      window.removeEventListener('pointerdown', close);
      window.removeEventListener('keydown', close);
    };
  }, [open]);
  return (
    <div className="open-menu" ref={menu}>
      <button type="button" className="chip primary open-button" aria-haspopup="menu" aria-expanded={open} disabled={props.vehicles === null} onClick={() => setOpen(!open)}>
        <span className="en">Open</span>
        <span className="zh">打开</span>
        <span className={`chevron ${open ? 'open' : ''}`} aria-hidden="true" />
      </button>
      {open && props.vehicles !== null && (
        <div className="menu" role="menu">
          <div className="menu-caption">
            Reference vehicles <span className="zh">参考车辆</span>
          </div>
          {props.vehicles.length === 0 ? (
            <div className="menu-empty">
              The server was started without reference vehicles. <span className="zh">服务器启动时未给出参考车辆。</span>
            </div>
          ) : (
            props.vehicles.map((vehicle) => (
              <button
                type="button"
                role="menuitem"
                key={vehicle.name}
                className={vehicle === props.current ? 'current' : ''}
                aria-current={vehicle === props.current || undefined}
                onClick={() => {
                  setOpen(false);
                  props.onOpen(vehicle);
                }}
              >
                <span className="en">{vehicle.title.en}</span>
                <span className="zh">{vehicle.title.zh}</span>
              </button>
            ))
          )}
        </div>
      )}
    </div>
  );
}

export function VehicleModelingTopBar(props: {
  vehicles: ReferenceVehicle[] | null;
  state: SessionState;
  onOpen: (vehicle: ReferenceVehicle) => void;
  onUndo: () => void;
  onRedo: () => void;
}) {
  const { state } = props;
  const opened = state.vehicle !== null;
  const status = checkStatus(state);
  return (
    <header className="top-bar">
      <div className="brand">
        <div className="tile">
          <span className="slashes">
            <i />
            <i />
            <i />
          </span>
          <span>ORVD</span>
        </div>
        <div className="titles">
          <div className="overline">
            ORVD · Modelling interface <span className="zh">建模界面</span>
          </div>
          <h1 className="title">
            Vehicle modelling
            <span className="slash">/</span>
            <span className="zh">车辆建模</span>
          </h1>
        </div>
      </div>
      <div className="top-vehicle">
        <OpenMenu vehicles={props.vehicles} current={state.vehicle?.reference ?? null} onOpen={props.onOpen} />
        <span className="meta">
          <span className="k">
            Vehicle <span className="zh">车辆</span>
          </span>
          <span className="v">{state.outline?.vehicleName ?? '—'}</span>
        </span>
        {state.opening !== null && (
          <span className="opening-chip" role="status">
            <span className="en">Opening {state.opening.title.en}</span>
            <span className="zh">正在打开{state.opening.title.zh}</span>
          </span>
        )}
      </div>
      <div className="top-session">
        <div className="history">
          <button type="button" className="chip" disabled={!state.canUndo || state.opening !== null} onClick={props.onUndo} title="Ctrl+Z">
            <span className="en">Undo</span>
            <span className="zh">撤销</span>
          </button>
          <button type="button" className="chip" disabled={!state.canRedo || state.opening !== null} onClick={props.onRedo} title="Ctrl+Shift+Z · Ctrl+Y">
            <span className="en">Redo</span>
            <span className="zh">重做</span>
          </button>
        </div>
        {opened && (
          <span className={`check-chip ${status}`} role="status">
            <span className="dot" />
            <span className="en">{checkStatusNames[status].en}</span>
            <span className="zh">{checkStatusNames[status].zh}</span>
          </span>
        )}
        <span className="meta">
          <span className="k">
            Revision <span className="zh">修订</span>
          </span>
          <span className="v">{opened ? state.revision : '—'}</span>
        </span>
      </div>
    </header>
  );
}
