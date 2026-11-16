import { sameElement, type ElementReference } from '../service/vehicle_modeling_service_protocol.ts';

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
 * Every kind the service describes, in its order, also a kind the vehicle has
 * no element of, so that one can be created. A filter shows only the kinds it
 * finds a row in, and opens them. The selected element keeps its row whatever
 * the filter says, so that a selection made elsewhere is always found.
 */
export function treeKinds(
  kinds: readonly string[],
  elements: readonly ElementReference[],
  filter: string,
  openKinds: ReadonlySet<string>,
  selection: ElementReference | null,
): TreeKind[] {
  const needle = filter.trim().toLowerCase();
  return kinds.flatMap((kind) => {
    const members = elements.filter((element) => element.kind === kind);
    const matching = members.filter((element) => element.name.toLowerCase().includes(needle));
    const rows = members.filter((element) => matching.includes(element) || sameElement(element, selection));
    if (needle !== '' && rows.length === 0) return [];
    return [{ kind, rows, total: members.length, matched: matching.length, open: needle !== '' || openKinds.has(kind) }];
  });
}
