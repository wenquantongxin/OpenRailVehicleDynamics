// The vehicle modelling server: serves the built interface and relays its
// commands to one resident orvd_vehicle_modeling_service.
//
// USAGE
//
//   npm run build
//   npm run serve:reference -- --service <path to orvd_vehicle_modeling_service>
//   open http://127.0.0.1:6651/            (?vehicle=GZ18 opens GZ18 on load,
//                                            &select=<kind>:<name> selects an element)
//
// For development, run the server as above and `npm run dev` beside it: Vite
// serves the interface on http://127.0.0.1:5183/ and proxies /api to port 6651.
//
// Options are strict key/value pairs:
//   --service PATH               the service binary (required)
//   --reference-vehicles FILE    the catalogue of reference vehicles the interface
//                                offers (see reference_vehicle_catalogue.ts);
//                                without it there are none
//   --host HOST                  default 127.0.0.1
//   --port PORT                  default 6651
// Paths given here are resolved against the working directory when the server
// starts; under npm that is this app's folder, which holds the catalogue of the
// repository's reference vehicles, reference_vehicles.json.
//
// HTTP
//   POST /api/command
//     The body is one request object of the service protocol (see
//     tools/vehicle_modeling_service/vehicle_modeling_service.cc). It is written
//     to the service as one line; the response body is the service's answer
//     line, unchanged. Requests are relayed strictly one at a time, in the order
//     they arrive. A request whose Origin names another host than its Host is
//     refused with 403, so that no page of another origin drives the service.
//   GET /api/reference_vehicles
//     [{"name", "title": {"en", "zh"}, "open_arguments": {...},
//       "visual_definition_available"}, ...] in catalogue order. open_arguments
//     are the arguments of the service's open_vehicle command for that vehicle;
//     the interface sends them as given and reads none of them.
//   GET /api/reference_vehicles/NAME/visual_definition
//     The vehicle's visual definition file, read when requested.
//   Anything else is a file of dist/.
//
// The service is started once. If it ends, the server reports why on standard
// error and stops: an interface cannot carry on without its project.

import { resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

import { readReferenceVehicleCatalogue } from './reference_vehicle_catalogue.ts';
import { startServer } from './server.ts';

export interface ServerArguments {
  servicePath: string;
  host: string;
  port: number;
  referenceVehiclesPath: string | null;
}

export function parseServerArguments(args: readonly string[]): ServerArguments {
  let servicePath: string | null = null;
  let host = '127.0.0.1';
  let port = 6651;
  let referenceVehiclesPath: string | null = null;
  const seen = new Set<string>();
  for (let index = 0; index < args.length; index += 2) {
    const key = args[index]!;
    const value = args[index + 1];
    if (value === undefined || value.startsWith('--')) throw new Error(`${key} needs a value`);
    if (seen.has(key)) throw new Error(`${key} is given twice`);
    seen.add(key);
    if (key === '--service') servicePath = resolve(value);
    else if (key === '--reference-vehicles') referenceVehiclesPath = resolve(value);
    else if (key === '--host') host = value;
    else if (key === '--port') {
      if (!/^\d+$/.test(value) || Number(value) < 1 || Number(value) > 65535) throw new Error('--port needs a port from 1 to 65535');
      port = Number(value);
    } else throw new Error(`unknown option ${key}`);
  }
  if (servicePath === null) throw new Error('--service PATH is required');
  return { servicePath, host, port, referenceVehiclesPath };
}

if (process.argv[1] !== undefined && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  try {
    const { servicePath, host, port, referenceVehiclesPath } = parseServerArguments(process.argv.slice(2));
    const referenceVehicles = referenceVehiclesPath === null ? [] : await readReferenceVehicleCatalogue(referenceVehiclesPath);
    const staticDirectory = fileURLToPath(new URL('../dist', import.meta.url));
    const server = await startServer({ servicePath, host, port, staticDirectory, referenceVehicles }, (ending) => {
      process.stderr.write(`The modelling service ${ending}; the modelling server stops.\n`);
      process.exit(1);
    });
    process.stdout.write(`Vehicle modelling: http://${host}:${server.port}/\n`);
  } catch (error) {
    process.stderr.write(`${error instanceof Error ? error.message : String(error)}\n`);
    process.exitCode = 1;
  }
}
