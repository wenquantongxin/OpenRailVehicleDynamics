import assert from 'node:assert/strict';
import { isAbsolute } from 'node:path';
import { test } from 'node:test';

import { parseServerArguments } from '../server/main.ts';

test('the service is required; the catalogue, host and port are optional, paths absolute', () => {
  const defaults = parseServerArguments(['--service', 'bin/service']);
  assert.ok(isAbsolute(defaults.servicePath));
  assert.deepEqual([defaults.host, defaults.port, defaults.referenceVehiclesPath], ['127.0.0.1', 6651, null]);
  const given = parseServerArguments(['--port', '7000', '--host', '0.0.0.0', '--reference-vehicles', 'catalogue.json', '--service', 'x']);
  assert.deepEqual([given.host, given.port], ['0.0.0.0', 7000]);
  assert.ok(given.referenceVehiclesPath !== null && isAbsolute(given.referenceVehiclesPath) && given.referenceVehiclesPath.endsWith('/catalogue.json'));
});

test('anything but complete, unrepeated key/value pairs is refused', () => {
  for (const args of [
    [],
    ['--service'],
    ['--service', '--port'],
    ['--service', 'x', '--service', 'y'],
    ['--service', 'x', '--port', '0'],
    ['--service', 'x', '--port', '80.5'],
    ['--service', 'x', '--colour', 'red'],
    ['--service', 'x', '--reference-vehicles', 'a.json', '--reference-vehicles', 'b.json'],
  ]) {
    assert.throws(() => parseServerArguments(args), Error, args.join(' '));
  }
});
