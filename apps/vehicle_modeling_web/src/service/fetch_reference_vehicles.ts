// The vehicles the modelling server offers, each with its title. Their open
// arguments are passed to the service's open_vehicle command as given: the
// interface reads none of them.

export interface ReferenceVehicle {
  /** The key of ?vehicle= links and of the visual definition's address. */
  name: string;
  title: { en: string; zh: string };
  openArguments: Readonly<Record<string, unknown>>;
  visualDefinitionAvailable: boolean;
}

async function fetchText(serverOrigin: string, path: string): Promise<string> {
  const response = await fetch(`${serverOrigin}${path}`);
  if (!response.ok) {
    throw new Error(`the modelling server answered ${path} with HTTP ${response.status}`);
  }
  return response.text();
}

export async function fetchReferenceVehicles(serverOrigin: string): Promise<ReferenceVehicle[]> {
  const listed: unknown = JSON.parse(await fetchText(serverOrigin, '/api/reference_vehicles'));
  if (!Array.isArray(listed)) {
    throw new Error('the reference vehicle list is not a list');
  }
  return listed.map((entry: { name?: unknown; title?: { en?: unknown; zh?: unknown }; open_arguments?: unknown; visual_definition_available?: unknown }) => {
    const { name, title, open_arguments: openArguments, visual_definition_available: visualDefinitionAvailable } = entry;
    if (
      typeof name !== 'string' ||
      typeof title?.en !== 'string' ||
      typeof title.zh !== 'string' ||
      typeof openArguments !== 'object' ||
      openArguments === null ||
      typeof visualDefinitionAvailable !== 'boolean'
    ) {
      throw new Error('a reference vehicle lacks its name, title, open arguments or visual definition availability');
    }
    return { name, title: { en: title.en, zh: title.zh }, openArguments: openArguments as Record<string, unknown>, visualDefinitionAvailable };
  });
}

export function fetchVisualDefinitionText(serverOrigin: string, vehicle: ReferenceVehicle): Promise<string> {
  return fetchText(serverOrigin, `/api/reference_vehicles/${encodeURIComponent(vehicle.name)}/visual_definition`);
}
