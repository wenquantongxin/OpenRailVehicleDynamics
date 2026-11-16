import { useEffect, useRef } from 'react';

import { CenteredModalDialog } from './CenteredModalDialog.tsx';
import type { BilingualText } from './open_choice_and_edited_project_wording.ts';

// The question before opening something else while the project has unsaved
// changes: they are lost if it is opened. Keeping on editing has the focus,
// and Escape answers with it.

export function DiscardUnsavedChangesDialog(props: { edited: BilingualText; chosen: BilingualText; onDiscard: () => void; onKeepEditing: () => void }) {
  const keep = useRef<HTMLButtonElement>(null);
  useEffect(() => {
    keep.current?.focus();
  }, []);
  return (
    <CenteredModalDialog en="Unsaved changes" zh="未保存的更改" className="discard-dialog" onDismiss={props.onKeepEditing}>
      <div className="dialog-body">
        <p className="removal-question">
          The changes to {props.edited.en} are not saved. Opening {props.chosen.en} loses them.
          <span className="zh">
            {props.edited.zh} 的更改尚未保存；打开{props.chosen.zh} 将丢失这些更改。
          </span>
        </p>
        <div className="form-buttons">
          <button type="button" className="chip danger" onClick={props.onDiscard}>
            <span className="en">Discard the changes and open</span>
            <span className="zh">放弃更改并打开</span>
          </button>
          <button type="button" className="chip" ref={keep} onClick={props.onKeepEditing}>
            <span className="en">Keep editing</span>
            <span className="zh">继续编辑</span>
          </button>
        </div>
      </div>
    </CenteredModalDialog>
  );
}
