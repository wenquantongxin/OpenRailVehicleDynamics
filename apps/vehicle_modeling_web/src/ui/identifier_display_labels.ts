// Labels for the identifiers the service sends. A kind or property identifier
// is read as words: underscores become spaces and a dot, which joins a record
// to its member, becomes a separator.

export function identifierLabel(identifier: string): string {
  return identifier.replaceAll('_', ' ').replaceAll('.', ' · ');
}
