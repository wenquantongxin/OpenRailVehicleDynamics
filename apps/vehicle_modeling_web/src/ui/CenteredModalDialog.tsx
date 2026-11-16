import { useEffect, useRef, type ReactNode } from 'react';

import { DockedPanelTitleStrip } from './DockedPanelTitleStrip.tsx';

// A dialog over the whole interface, which takes no input until it is
// answered: the browser's own modal dialog, with a title strip like the
// panels'. Escape dismisses it.

export function CenteredModalDialog(props: { en: string; zh: string; className: string; onDismiss: () => void; children: ReactNode }) {
  const dialog = useRef<HTMLDialogElement>(null);
  useEffect(() => {
    const shown = dialog.current!;
    shown.showModal();
    return () => shown.close();
  }, []);
  return (
    <dialog
      ref={dialog}
      className={`modal-dialog ${props.className}`}
      aria-label={props.en}
      onCancel={(event) => {
        event.preventDefault();
        props.onDismiss();
      }}
    >
      <DockedPanelTitleStrip en={props.en} zh={props.zh} />
      {props.children}
    </dialog>
  );
}
