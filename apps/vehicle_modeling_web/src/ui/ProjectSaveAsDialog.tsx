import { useEffect, useId, useRef, useState } from 'react';

import type { SaveAsDialog } from '../session/vehicle_editing_session.ts';
import { CenteredModalDialog } from './CenteredModalDialog.tsx';
import { FieldRefusalMessage } from './FieldRefusalMessage.tsx';

// Save as: one name, the project's own to begin with. The service says
// whether a name is acceptable, and its refusal stays at the field with what
// was typed. A name that is another project of the folder is asked about
// before it is replaced; the safe answer has the focus. While the folder is
// listed and the project written, the dialog cannot be left: what is on its
// way is answered first.

export function ProjectSaveAsDialog(props: {
  dialog: SaveAsDialog;
  vehicleProjectsDirectory: string;
  onSave: (name: string, replacingAnotherProject: boolean) => void;
  onChooseAnotherName: () => void;
  onCancel: () => void;
}) {
  const { dialog } = props;
  const [name, setName] = useState(dialog.name);
  const input = useRef<HTMLInputElement>(null);
  const keep = useRef<HTMLButtonElement>(null);
  const refusalId = useId();
  const takesTyping = !dialog.saving && !dialog.replaceQuestion;

  useEffect(() => {
    if (dialog.replaceQuestion) keep.current?.focus();
    else if (!dialog.saving) input.current?.focus();
  }, [dialog.replaceQuestion, dialog.saving]);
  // Declared after the focus, so that on opening the name is focused and then all selected: typing replaces it.
  useEffect(() => {
    input.current?.select();
  }, []);

  return (
    <CenteredModalDialog
      en="Save as"
      zh="另存为"
      className="save-as-dialog"
      onDismiss={() => {
        if (!dialog.saving) props.onCancel();
      }}
    >
      <form
        className="dialog-body"
        onSubmit={(event) => {
          event.preventDefault();
          if (takesTyping) props.onSave(name, false);
        }}
      >
        <p className="dialog-note">
          The project is saved in the folder of projects under this name, and the vehicle takes the name too.
          <span className="zh">工程以此名称保存到工程文件夹，车辆也随之改用此名称。</span>
        </p>
        <label className="dialog-field">
          <span className="dialog-field-label">
            Project name <span className="zh">工程名称</span>
          </span>
          <span className="text-field">
            <input
              ref={input}
              className="project-name-input"
              value={name}
              readOnly={!takesTyping}
              spellCheck={false}
              autoComplete="off"
              aria-invalid={dialog.refusal !== null}
              aria-describedby={dialog.refusal === null ? undefined : refusalId}
              onChange={(event) => setName(event.target.value)}
            />
          </span>
        </label>
        {dialog.refusal !== null && (
          <div id={refusalId}>
            <FieldRefusalMessage message={dialog.refusal} />
          </div>
        )}
        <p className="dialog-folder">
          Folder <span className="zh">文件夹</span> <span className="projects-folder">{props.vehicleProjectsDirectory}</span>
        </p>
        {dialog.replaceQuestion ? (
          <div className="replace-question">
            <p className="removal-question">
              <b>{dialog.name}</b> is already a project in the folder. Saving replaces it with this one.
              <span className="zh">文件夹中已有工程 {dialog.name}；保存将以当前工程替换它。</span>
            </p>
            <div className="form-buttons">
              <button type="button" className="chip danger" onClick={() => props.onSave(dialog.name, true)}>
                <span className="en">Replace {dialog.name}</span>
                <span className="zh">替换</span>
              </button>
              <button type="button" className="chip" ref={keep} onClick={props.onChooseAnotherName}>
                <span className="en">Choose another name</span>
                <span className="zh">换个名称</span>
              </button>
            </div>
          </div>
        ) : (
          <div className="form-buttons">
            <button type="submit" className="chip primary" disabled={dialog.saving}>
              <span className="en">{dialog.saving ? 'Saving' : 'Save'}</span>
              <span className="zh">{dialog.saving ? '正在保存' : '保存'}</span>
            </button>
            <button type="button" className="chip" disabled={dialog.saving} onClick={props.onCancel}>
              <span className="en">Cancel</span>
              <span className="zh">取消</span>
            </button>
          </div>
        )}
      </form>
    </CenteredModalDialog>
  );
}
