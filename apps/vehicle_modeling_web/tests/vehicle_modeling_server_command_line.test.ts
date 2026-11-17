import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { access, mkdtemp, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { isAbsolute, join } from 'node:path';
import { test } from 'node:test';
import { fileURLToPath } from 'node:url';

import { parseVehicleModelingServerArguments } from '../server/vehicle_modeling_server_command_line.ts';

test('the service and the folder of projects are required; the catalogue, host and port are optional, paths absolute', () => {
  const defaults = parseVehicleModelingServerArguments(['--service', 'bin/service', '--vehicle-projects', 'projects']);
  assert.ok(isAbsolute(defaults.servicePath));
  assert.ok(isAbsolute(defaults.vehicleProjectsDirectory) && defaults.vehicleProjectsDirectory.endsWith('/projects'));
  assert.deepEqual([defaults.host, defaults.port, defaults.startingPointsPath], ['127.0.0.1', 6651, null]);
  const given = parseVehicleModelingServerArguments(['--port', '7000', '--host', '0.0.0.0', '--starting-points', 'catalogue.json', '--vehicle-projects', '/p', '--service', 'x']);
  assert.deepEqual([given.host, given.port, given.vehicleProjectsDirectory], ['0.0.0.0', 7000, '/p']);
  assert.ok(given.startingPointsPath !== null && isAbsolute(given.startingPointsPath) && given.startingPointsPath.endsWith('/catalogue.json'));
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
    ['--vehicle-projects', 'p', '--service', 'x', '--starting-points', 'a.json', '--starting-points', 'b.json'],
    ['--vehicle-projects', 'p', '--service', 'x', '--reference-vehicles', 'a.json'],
  ]) {
    assert.throws(() => parseVehicleModelingServerArguments(args), Error, args.join(' '));
  }
});

test('without a folder of projects the server is not started, and says so', () => {
  assert.throws(() => parseVehicleModelingServerArguments(['--service', 'x']), /--vehicle-projects DIRECTORY is required/);
});

test('run without a terminal, the server refuses a missing folder of projects without a question, ends with code 1 and creates nothing', { timeout: 20000 }, async () => {
  const directory = await mkdtemp(join(tmpdir(), 'orvd-modelling-command-line-'));
  try {
    const missing = join(directory, 'absent', 'projects');
    const commandLine = fileURLToPath(new URL('../server/vehicle_modeling_server_command_line.ts', import.meta.url));
    // Standard input and output are pipes here, as for a script.
    const server = spawn(process.execPath, ['--experimental-strip-types', '--no-warnings', commandLine, '--service', join(directory, 'service'), '--vehicle-projects', missing, '--port', '6684'], {
      stdio: ['pipe', 'pipe', 'pipe'],
    });
    let output = '';
    let errors = '';
    server.stdout.on('data', (chunk) => (output += chunk));
    server.stderr.on('data', (chunk) => (errors += chunk));
    // A server that started after all is stopped, so that the test fails instead of waiting.
    const stopper = setTimeout(() => server.kill(), 10000);
    const code = await new Promise((ended) => server.once('exit', ended));
    clearTimeout(stopper);
    assert.equal(code, 1);
    assert.equal(output, '', 'nothing is asked');
    assert.match(errors, new RegExp(`the folder of vehicle projects '${missing}' is not an existing directory`));
    await assert.rejects(access(join(directory, 'absent')), /ENOENT/, 'nothing is created');
  } finally {
    await rm(directory, { recursive: true, force: true });
  }
});
