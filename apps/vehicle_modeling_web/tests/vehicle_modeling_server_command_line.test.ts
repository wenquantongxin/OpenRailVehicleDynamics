import assert from 'node:assert/strict';
import { isAbsolute } from 'node:path';
import { test } from 'node:test';

import { parseVehicleModelingServerArguments } from '../server/vehicle_modeling_server_command_line.ts';

test('the service and the folder of projects are required; the catalogue, host and port are optional, paths absolute', () => {
  const defaults = parseVehicleModelingServerArguments(['--service', 'bin/service', '--vehicle-projects', 'projects']);
  assert.ok(isAbsolute(defaults.servicePath));
  assert.ok(isAbsolute(defaults.vehicleProjectsDirectory) && defaults.vehicleProjectsDirectory.endsWith('/projects'));
  assert.deepEqual([defaults.host, defaults.port, defaults.referenceVehiclesPath], ['127.0.0.1', 6651, null]);
  const given = parseVehicleModelingServerArguments(['--port', '7000', '--host', '0.0.0.0', '--reference-vehicles', 'catalogue.json', '--vehicle-projects', '/p', '--service', 'x']);
  assert.deepEqual([given.host, given.port, given.vehicleProjectsDirectory], ['0.0.0.0', 7000, '/p']);
  assert.ok(given.referenceVehiclesPath !== null && isAbsolute(given.referenceVehiclesPath) && given.referenceVehiclesPath.endsWith('/catalogue.json'));
});

test('anything but complete, unrepeated key/value pairs is refused', () => {
  for (const args of [
    [],
    ['--service'],
    ['--service', '--port'],
    ['--vehicle-projects', 'p', '--service', 'x', '--service', 'y'],
    ['--vehicle-projects', 'p', '--service', 'x', '--port', '0'],
    ['--vehicle-projects', 'p', '--service', 'x', '--port', '80.5'],
    ['--vehicle-projects', 'p', '--service', 'x', '--colour', 'red'],
    ['--vehicle-projects', 'p', '--service', 'x', '--reference-vehicles', 'a.json', '--reference-vehicles', 'b.json'],
  ]) {
    assert.throws(() => parseVehicleModelingServerArguments(args), Error, args.join(' '));
  }
});

test('without a folder of projects the server is not started, and says so', () => {
  assert.throws(() => parseVehicleModelingServerArguments(['--service', 'x']), /--vehicle-projects DIRECTORY is required/);
});
