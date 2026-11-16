import { sameElement, type ElementReference } from '../service/protocol.ts';

// Which rows the component tree shows, apart from its component so that the
// rule can be tested without a page.

export interface TreeKind {
  kind: string;
  /** The rows shown under the kind when it is open. */
  rows: ElementReference[];
  total: number;
  /** How many elements match the filter; all of them without one. */
  matched: number;
  open: boolean;
}

/**
 * The kinds in the outline's order; a kind with no row to show is left out. A
 * filter opens every kind it shows. The selected element keeps its row
 * whatever the filter says, so that a selection made elsewhere is always found.
 */
export function treeKinds(elements: readonly ElementReference[], filter: string, openKinds: ReadonlySet<string>, selection: ElementReference | null): TreeKind[] {
  const needle = filter.trim().toLowerCase();
  const grouped = new Map<string, ElementReference[]>();
  for (const element of elements) {
    grouped.set(element.kind, [...(grouped.get(element.kind) ?? []), element]);
  }
  return [...grouped].flatMap(([kind, members]) => {
    const matching = members.filter((element) => element.name.toLowerCase().includes(needle));
    const rows = members.filter((element) => matching.includes(element) || sameElement(element, selection));
    return rows.length === 0 ? [] : [{ kind, rows, total: members.length, matched: matching.length, open: needle !== '' || openKinds.has(kind) }];
  });
}
