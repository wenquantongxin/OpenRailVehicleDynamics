import { checkStatus, type SessionState } from '../session/vehicle_editing_session.ts';
import { openChoiceName } from './open_choice_and_edited_project_wording.ts';

// Title; Open, Save and Save as; what is being edited: the project's name when
// the session has a file, otherwise the vehicle's with a plain statement that
// it is not saved to a file, and a mark while there are unsaved changes; undo
// and redo; and the check status with the project revision.

const checkStatusNames = {
  passed: { en: 'Check passed', zh: '检查通过' },
  failed: { en: 'Check failed', zh: '检查未通过' },
  not_checked: { en: 'Not checked', zh: '未检查' },
};

export function VehicleModelingTopBar(props: {
  state: SessionState;
  onShowOpenChoices: () => void;
  onSave: () => void;
  onSaveAs: () => void;
  onUndo: () => void;
  onRedo: () => void;
}) {
  const { state } = props;
  const opened = state.vehicle !== null;
  const busy = state.opening !== null || state.failure !== null;
  const file = state.vehicleProjectFile;
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
        <div className="file-actions">
          <button type="button" className="chip" disabled={!opened || busy || state.choosingWhatToOpen} onClick={props.onShowOpenChoices}>
            <span className="en">Open</span>
            <span className="zh">打开</span>
          </button>
          <button type="button" className="chip primary" disabled={!opened || busy} onClick={props.onSave} title="Ctrl+S">
            <span className="en">Save</span>
            <span className="zh">保存</span>
          </button>
          <button type="button" className="chip" disabled={!opened || busy} onClick={props.onSaveAs}>
            <span className="en">Save as</span>
            <span className="zh">另存为</span>
          </button>
        </div>
        <span className="meta edited-project">
          {file !== null ? (
            <span className="k">
              Project <span className="zh">工程</span>
            </span>
          ) : opened ? (
            <span className="k">
              Vehicle, not saved to a file <span className="zh">车辆，未保存为文件</span>
            </span>
          ) : (
            <span className="k">
              Vehicle <span className="zh">车辆</span>
            </span>
          )}
          <span className="v">
            <span className="edited-name">{file?.name ?? state.outline?.vehicleName ?? '—'}</span>
            {opened && state.hasUnsavedChanges && (
              <span className="unsaved-mark" role="status">
                <span className="dot" />
                <span className="en">Unsaved changes</span>
                <span className="zh">未保存的更改</span>
              </span>
            )}
          </span>
        </span>
        {state.opening !== null && (
          <span className="opening-chip" role="status">
            <span className="en">Opening {openChoiceName(state.opening)}</span>
            <span className="zh">正在打开 {openChoiceName(state.opening)}</span>
          </span>
        )}
      </div>
      <div className="top-session">
        <div className="history">
          <button type="button" className="chip" disabled={!state.canUndo || busy} onClick={props.onUndo} title="Ctrl+Z">
            <span className="en">Undo</span>
            <span className="zh">撤销</span>
          </button>
          <button type="button" className="chip" disabled={!state.canRedo || busy} onClick={props.onRedo} title="Ctrl+Shift+Z · Ctrl+Y">
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
