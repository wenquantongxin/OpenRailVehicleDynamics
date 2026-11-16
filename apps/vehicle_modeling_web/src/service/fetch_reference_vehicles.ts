// The vehicles the modelling server offers, each with its title. Their open
// arguments are passed to the service's open_vehicle command as given: the
// interface reads none of them.

export interface ReferenceVehicle {
  /** The key of ?vehicle= links. */
  name: string;
  title: { en: string; zh: string };
  openArguments: Readonly<Record<string, unknown>>;
}

export async function fetchReferenceVehicles(serverOrigin: string): Promise<ReferenceVehicle[]> {
  const response = await fetch(`${serverOrigin}/api/reference_vehicles`);
  if (!response.ok) {
    throw new Error(`the modelling server answered /api/reference_vehicles with HTTP ${response.status}`);
  }
  const listed: unknown = await response.json();
  if (!Array.isArray(listed)) {
    throw new Error('the reference vehicle list is not a list');
  }
  return listed.map((entry: { name?: unknown; title?: { en?: unknown; zh?: unknown }; open_arguments?: unknown }) => {
    const { name, title, open_arguments: openArguments } = entry;
    if (typeof name !== 'string' || typeof title?.en !== 'string' || typeof title.zh !== 'string' || typeof openArguments !== 'object' || openArguments === null) {
      throw new Error('a reference vehicle lacks its name, title or open arguments');
    }
    return { name, title: { en: title.en, zh: title.zh }, openArguments: openArguments as Record<string, unknown> };
  });
}
