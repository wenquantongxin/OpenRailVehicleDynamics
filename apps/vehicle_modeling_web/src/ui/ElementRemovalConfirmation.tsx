import { useEffect, useId, useRef } from 'react';

import type { PendingRemoval } from '../session/vehicle_editing_session.ts';
import { ElementsRemovedWithElementList } from './ElementsRemovedWithElementList.tsx';
import { identifierLabel } from './identifier_display_labels.ts';

// The question every removal asks: what would go, and a choice between
// removing and keeping it. An element nothing refers to goes alone, and the
// question says so; otherwise everything that goes with it is listed. Cancel
// has the focus, and Escape cancels.

export function ElementRemovalConfirmation(props: { removal: PendingRemoval; onConfirm: () => void; onCancel: () => void }) {
  const { element, removedElements } = props.removal;
  const alone = removedElements.length === 1;
  const cancel = useRef<HTMLButtonElement>(null);
  const questionId = useId();
  useEffect(() => {
    cancel.current?.focus({ preventScroll: true });
  }, []);
  return (
    <section
      className="removal-confirmation"
      role="alertdialog"
      aria-labelledby={questionId}
      onKeyDown={(event) => {
        if (event.key === 'Escape') props.onCancel();
      }}
    >
      <div id={questionId} className="removal-what">
        {alone ? (
          <>
            <p className="removal-question">
              Remove this {identifierLabel(element.kind)}? Nothing refers to it, so it goes alone.
              <span className="zh">删除该元素？没有其他元素引用它，只删除它本身。</span>
            </p>
            <p className="removal-note">
              Undo brings it back. <span className="zh">撤销可恢复。</span>
            </p>
          </>
        ) : (
          <ElementsRemovedWithElementList element={element} removedElements={removedElements} />
        )}
      </div>
      <div className="form-buttons">
        <button type="button" className="chip danger" onClick={props.onConfirm}>
          <span className="en">{alone ? 'Remove' : `Remove all ${removedElements.length}`}</span>
          <span className="zh">{alone ? '删除' : '全部删除'}</span>
        </button>
        <button type="button" className="chip" ref={cancel} onClick={props.onCancel}>
          <span className="en">Cancel</span>
          <span className="zh">取消</span>
        </button>
      </div>
    </section>
  );
}
