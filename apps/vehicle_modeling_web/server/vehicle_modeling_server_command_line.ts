// The modelling server: serves the built interface and relays its commands
// to one resident orvd_vehicle_modeling_service.
//
// USAGE
//
//   npm run build
//   npm run serve:starting-points -- --service <path to orvd_vehicle_modeling_service>
//                                    --vehicle-projects <folder of projects>
//   open http://127.0.0.1:6651/            (a session the service has open is
//                                            resumed; otherwise
//                                            ?starting_point=NAME starts a new
//                                            project from that starting point,
//                                            ?project=NAME opens a saved
//                                            project, and then
//                                            &select=<kind>:<name> selects an element)
//
// For development, run the server as above and `npm run dev` beside it: Vite
// serves the interface on http://127.0.0.1:5183/ and proxies /api to port 6651.
//
// Options are strict key/value pairs:
//   --service PATH               the service binary (required)
//   --vehicle-projects DIRECTORY the folder the projects are saved in and opened
//                                from, outside the repository (required); when
//                                it does not exist, a server run in a terminal
//                                asks whether to create it, and one run
//                                without a terminal refuses to start
//   --starting-points FILE       the catalogue of the starting points a new
//                                project can start from (see
//                                read_project_starting_point_catalogue.ts);
//                                without it there are none
//   --host HOST                  default 127.0.0.1
//   --port PORT                  default 6651
// Paths given here are resolved against the working directory when the server
// starts; under npm that is this app's folder, which holds the catalogue of the
// repository's starting points, project_starting_point_catalogue.json.
//
// HTTP
//   POST /api/command
//     The body is one request object of the service protocol (see
//     tools/vehicle_modeling_service/vehicle_modeling_service.cc). It is written
//     to the service as one line; the response body is the service's answer
//     line, unchanged. Requests are relayed strictly one at a time, in the order
//     they arrive. A request whose Origin names another host than its Host is
//     refused with 403, so that no page of another origin drives the service.
//   GET /api/project_starting_points
//     [{"name", "title", "description", "category", "open": {"command",
//     "arguments"}}, ...] in catalogue order, each text {"en", "zh"}. "open" is
//     the service command that starts a new project from the starting point,
//     new_project_from_template for a template and open_vehicle for a
//     vehicle's own files, with its arguments, their paths absolute; the
//     interface sends them as given and reads none of them.
//   GET /api/vehicle_projects_directory
//     {"directory": "<absolute path>"}: the folder of projects, which the
//     interface passes to the service's commands that take a folder.
//   Anything else is a file of dist/.
//
// The service is started once. If it ends, the server reports why on standard
// error and stops: an interface cannot carry on without its project.

import { resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

import { readProjectStartingPointCatalogue } from './read_project_starting_point_catalogue.ts';
import { terminalConversation } from './terminal_conversation.ts';
import { startVehicleModelingHttpServer } from './vehicle_modeling_http_server.ts';

export interface VehicleModelingServerArguments {
  servicePath: string;
  vehicleProjectsDirectory: string;
  host: string;
  port: number;
  startingPointsPath: string | null;
}

export function parseVehicleModelingServerArguments(args: readonly string[]): VehicleModelingServerArguments {
  let servicePath: string | null = null;
  let vehicleProjectsDirectory: string | null = null;
  let host = '127.0.0.1';
  let port = 6651;
  let startingPointsPath: string | null = null;
  const seen = new Set<string>();
  for (let index = 0; index < args.length; index += 2) {
    const key = args[index]!;
    const value = args[index + 1];
    if (value === undefined || value.startsWith('--')) throw new Error(`${key} needs a value`);
    if (seen.has(key)) throw new Error(`${key} is given twice`);
    seen.add(key);
    if (key === '--service') servicePath = resolve(value);
    else if (key === '--vehicle-projects') vehicleProjectsDirectory = resolve(value);
    else if (key === '--starting-points') startingPointsPath = resolve(value);
    else if (key === '--host') host = value;
    else if (key === '--port') {
      if (!/^\d+$/.test(value) || Number(value) < 1 || Number(value) > 65535) throw new Error('--port needs a port from 1 to 65535');
      port = Number(value);
    } else throw new Error(`unknown option ${key}`);
  }
  if (servicePath === null) throw new Error('--service PATH is required');
  if (vehicleProjectsDirectory === null) throw new Error('--vehicle-projects DIRECTORY is required: the folder the projects are saved in');
  return { servicePath, vehicleProjectsDirectory, host, port, startingPointsPath };
}

if (process.argv[1] !== undefined && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  try {
    const { servicePath, vehicleProjectsDirectory, host, port, startingPointsPath } = parseVehicleModelingServerArguments(process.argv.slice(2));
    const startingPoints = startingPointsPath === null ? [] : await readProjectStartingPointCatalogue(startingPointsPath);
    const staticDirectory = fileURLToPath(new URL('../dist', import.meta.url));
    const terminal = terminalConversation();
    const server = await startVehicleModelingHttpServer({ servicePath, host, port, staticDirectory, startingPoints, vehicleProjectsDirectory, terminal }, (ending) => {
      process.stderr.write(`The modelling service ${ending}; the modelling server stops.\n`);
      process.exit(1);
    });
    process.stdout.write(`Multibody modelling: http://${host}:${server.port}/\n`);
  } catch (error) {
    process.stderr.write(`${error instanceof Error ? error.message : String(error)}\n`);
    process.exitCode = 1;
  }
}
