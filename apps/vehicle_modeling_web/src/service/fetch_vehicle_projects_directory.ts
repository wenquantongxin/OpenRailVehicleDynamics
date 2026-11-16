/**
 * The folder of projects the modelling server was started with. The interface
 * hands the string to the service's commands as it is and does nothing else
 * with it: which file a project is, the service says.
 */
export async function fetchVehicleProjectsDirectory(serverOrigin: string): Promise<string> {
  const response = await fetch(`${serverOrigin}/api/vehicle_projects_directory`);
  if (!response.ok) {
    throw new Error(`the modelling server answered /api/vehicle_projects_directory with HTTP ${response.status}`);
  }
  const { directory } = (await response.json()) as { directory?: unknown };
  if (typeof directory !== 'string') {
    throw new Error('the modelling server named no folder of projects');
  }
  return directory;
}
