import { rowPathOfSubject } from '../service/find_model_tree_rows.ts';
import { sameSubject, subjectElement, type ElementReference, type ModelTreeNode, type PropertySubject } from '../service/vehicle_modeling_service_protocol.ts';

// The model tree's display rules, apart from its components so that they can
// be tested without a page: which rows are open when a vehicle opens and which
// stay open when the tree is read again, how many element rows a row has
// directly under it, when an element row shows its kind as a tag, which rows
// have a "+", what a filter by name shows, and which rows open to reveal a
// selection. A row is known by its key and by what the service says of it,
// never by what its title says.

/** Whether a row is an element's row. */
export function isElementRow(row: ModelTreeNode): boolean {
  return subjectElement(row.subject) !== null;
}

/**
 * The rows open when a vehicle opens: the first first-level row that lists
 * elements, so that a vehicle opens on its first list of elements, with that
 * list's own rows closed; every other first-level row closed.
 */
export function rowsOpenWhenAVehicleOpens(tree: readonly ModelTreeNode[]): ReadonlySet<string> {
  const first = tree.find((row) => row.children.some(isElementRow));
  return new Set(first === undefined ? [] : [first.key]);
}

/**
 * The open rows, carried over a new reading of the tree. A row keeps its key
 * across edits, except that renaming an element renames the keys of its row
 * and of the rows under it: the renamed row is the one standing where the old
 * one stood, a row of the same kind at the same place among its parent's rows,
 * which have kept their number, and the rows under it are found likewise.
 */
export function openRowsCarriedOver(previous: readonly ModelTreeNode[], next: readonly ModelTreeNode[], open: ReadonlySet<string>): ReadonlySet<string> {
  const renamed = new Map<string, string>();
  const match = (before: readonly ModelTreeNode[], after: readonly ModelTreeNode[]): void => {
    const beforeByKey = new Map(before.map((row) => [row.key, row]));
    const afterKeys = new Set(after.map((row) => row.key));
    after.forEach((row, place) => {
      const same = beforeByKey.get(row.key);
      if (same !== undefined) {
        match(same.children, row.children);
        return;
      }
      const old = before.length === after.length ? before[place] : undefined;
      if (old === undefined || afterKeys.has(old.key) || !sameKindOfRow(old, row)) return;
      renamed.set(old.key, row.key);
      match(old.children, row.children);
    });
  };
  match(previous, next);
  return new Set([...open].map((key) => renamed.get(key) ?? key));
}

/** Whether two rows are of one kind: element rows of the same element kind, or rows of the same title selecting the same or nothing. */
function sameKindOfRow(a: ModelTreeNode, b: ModelTreeNode): boolean {
  const elementA = subjectElement(a.subject);
  const elementB = subjectElement(b.subject);
  if (elementA !== null || elementB !== null) return elementA !== null && elementB !== null && elementA.kind === elementB.kind;
  const sameTitle = a.title !== null && b.title !== null && a.title.en === b.title.en && a.title.zh === b.title.zh;
  return sameTitle && ((a.subject === null && b.subject === null) || sameSubject(a.subject, b.subject));
}

/** How many element rows stand directly under the row; those deeper are counted by the rows they stand under. */
export function elementRowsDirectlyUnder(row: ModelTreeNode): number {
  return row.children.filter(isElementRow).length;
}

/** Whether a row states how many element rows stand directly under it: a titled row, at any level, that has any. */
export function showsElementCount(row: ModelTreeNode): boolean {
  return !isElementRow(row) && elementRowsDirectlyUnder(row) > 0;
}

/**
 * Whether an element row shows its kind as a tag: in a list of types, one
 * whose row offers several kinds to create, and wherever an element of one of
 * the kinds bodies are attached by is listed, when there are several such
 * kinds, for their row offers them only through the body it attaches.
 */
export function showsKindTag(parent: ModelTreeNode | null, element: ElementReference, attachingKinds: readonly string[]): boolean {
  return parent !== null && (parent.creates.length > 1 || (attachingKinds.length > 1 && attachingKinds.includes(element.kind)));
}

/** Whether a row has a "+": it creates something, or attaches one of the bodies it lists. */
export function hasCreateButton(row: ModelTreeNode): boolean {
  return row.provided && (row.creates.length > 0 || row.attaches.length > 0);
}

/** The rows above the subject's row, from the first level down, which open to reveal it; none when the tree has no such row. */
export function rowsAboveSubject(tree: readonly ModelTreeNode[], subject: PropertySubject): string[] {
  const path = rowPathOfSubject(tree, subject);
  return path === null ? [] : path.slice(0, -1).map((row) => row.key);
}

/** What a filter by name shows. */
export interface RowsShownByFilter {
  /** The element rows whose name holds the text. */
  matching: ReadonlySet<string>;
  /** Those rows, the selected one, and every row above them. */
  shown: ReadonlySet<string>;
  /** The rows above them, which the filter opens. */
  open: ReadonlySet<string>;
}

/**
 * Every element row whose name holds the text, whatever the case, with the
 * rows above it, opened. The selected row stays shown, so that a selection
 * made elsewhere is always found.
 */
export function rowsShownByFilter(tree: readonly ModelTreeNode[], filter: string, selection: PropertySubject | null): RowsShownByFilter {
  const needle = filter.trim().toLowerCase();
  const matching = new Set<string>();
  const shown = new Set<string>();
  const open = new Set<string>();
  const visit = (rows: readonly ModelTreeNode[], above: readonly ModelTreeNode[]): void => {
    for (const row of rows) {
      const element = subjectElement(row.subject);
      const matches = element !== null && element.name.toLowerCase().includes(needle);
      if (matches) matching.add(row.key);
      if (matches || sameSubject(row.subject, selection)) {
        shown.add(row.key);
        for (const each of above) {
          shown.add(each.key);
          open.add(each.key);
        }
      }
      visit(row.children, [...above, row]);
    }
  };
  visit(tree, []);
  return { matching, shown, open };
}

/** How many of the element rows directly under the row the filter matches. */
export function matchingElementRowsDirectlyUnder(row: ModelTreeNode, matching: ReadonlySet<string>): number {
  return row.children.filter((child) => matching.has(child.key)).length;
}
