import type { BilingualTitle } from './vehicle_modeling_service_protocol.ts';

// The starting points the modelling server offers a new project, each with its
// title, its line of description and its category, in the server's order.
// Each comes with the service command that starts a new project from it and
// the command's arguments, which are sent as given: the interface reads none
// of them and knows no starting point.

export interface ProjectStartingPoint {
  /** The key of ?starting_point= links. */
  name: string;
  title: BilingualTitle;
  description: BilingualTitle;
  /** Only helps to find a starting point. */
  category: BilingualTitle;
  openCommand: string;
  openArguments: Readonly<Record<string, unknown>>;
}

function bilingual(value: unknown): BilingualTitle | null {
  const title = value as { en?: unknown; zh?: unknown } | null;
  return typeof title?.en === 'string' && typeof title.zh === 'string' ? { en: title.en, zh: title.zh } : null;
}

export async function fetchProjectStartingPoints(serverOrigin: string): Promise<ProjectStartingPoint[]> {
  const response = await fetch(`${serverOrigin}/api/project_starting_points`);
  if (!response.ok) {
    throw new Error(`the modelling server answered /api/project_starting_points with HTTP ${response.status}`);
  }
  const listed: unknown = await response.json();
  if (!Array.isArray(listed)) {
    throw new Error('the list of starting points is not a list');
  }
  return listed.map((entry: { name?: unknown; title?: unknown; description?: unknown; category?: unknown; open?: { command?: unknown; arguments?: unknown } }) => {
    const [title, description, category] = [bilingual(entry.title), bilingual(entry.description), bilingual(entry.category)];
    const openArguments = entry.open?.arguments;
    if (typeof entry.name !== 'string' || title === null || description === null || category === null || typeof entry.open?.command !== 'string' || typeof openArguments !== 'object' || openArguments === null) {
      throw new Error('a starting point lacks its name, title, description, category or the command that opens it');
    }
    return { name: entry.name, title, description, category, openCommand: entry.open.command, openArguments: openArguments as Record<string, unknown> };
  });
}
