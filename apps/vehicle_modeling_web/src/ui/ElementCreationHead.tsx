import { identifierLabel } from './identifier_display_labels.ts';

// The head of the property panel while it creates an element of a kind.

export function ElementCreationHead({ kind }: { kind: string }) {
  return (
    <div className="selection-head creation-head">
      <div className="selection-title">
        <span className="selection-kind">
          New element <span className="zh">新建元素</span>
        </span>
        <span className="creation-kind">{identifierLabel(kind)}</span>
      </div>
    </div>
  );
}
