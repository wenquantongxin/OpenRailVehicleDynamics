// Which choice a token field shows, apart from its component so that the rule
// can be tested without a page. A value that is none of the choices, such as a
// token a record was created without, shows no choice, so that every choice,
// the first included, can be chosen.

/** The choice the field shows; undefined shows none. */
export function shownChoice(value: string | null | undefined, choices: readonly string[]): string | undefined {
  return value !== null && value !== undefined && choices.includes(value) ? value : undefined;
}
