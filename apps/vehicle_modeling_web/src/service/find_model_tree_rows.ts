import { sameSubject, type ModelTreeNode, type PropertySubject } from './vehicle_modeling_service_protocol.ts';

// Finding rows of the model tree: a subject's by what its row selects, and a
// row by its key; never by what a row is called or where the interface
// expects it.

/** The row that selects the subject, after the rows above it from the first level down; null when the tree has no such row. */
export function rowPathOfSubject(tree: readonly ModelTreeNode[], subject: PropertySubject): ModelTreeNode[] | null {
  for (const row of tree) {
    if (sameSubject(row.subject, subject)) return [row];
    const below = rowPathOfSubject(row.children, subject);
    if (below !== null) return [row, ...below];
  }
  return null;
}

export function treeHasSubject(tree: readonly ModelTreeNode[] | null, subject: PropertySubject): boolean {
  return tree !== null && rowPathOfSubject(tree, subject) !== null;
}

/** The row with the key, at any depth; null when the tree has none. */
export function rowWithKey(tree: readonly ModelTreeNode[], key: string): ModelTreeNode | null {
  for (const row of tree) {
    if (row.key === key) return row;
    const below = rowWithKey(row.children, key);
    if (below !== null) return below;
  }
  return null;
}
