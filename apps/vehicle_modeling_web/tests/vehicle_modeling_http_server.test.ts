import assert from 'node:assert/strict';
import { access, chmod, mkdir, mkdtemp, readFile, rm, stat, writeFile } from 'node:fs/promises';
import type { AddressInfo } from 'node:net';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { test } from 'node:test';

import { createServer as createViteServer } from 'vite';

import type { ReferenceVehicleCatalogueEntry } from '../server/read_reference_vehicle_catalogue.ts';
import type { TerminalConversation } from '../server/terminal_conversation.ts';
import { startVehicleModelingHttpServer } from '../server/vehicle_modeling_http_server.ts';
import { modellingServerProxy } from '../vite.config.ts';

// The server against a stand-in service: a child process that notes that it
// started, answers each line after a delay the request chooses, says how many
// requests it held when one arrived and how many it has received, and ends
// with a chosen exit code on request.

const fakeServiceSource = `
require('node:fs').appendFileSync(require('node:path').join(__dirname, 'started.txt'), 'started\\n');
const lines = require('node:readline').createInterface({ input: process.stdin });
let held = 0;
let received = 0;
lines.on('line', (line) => {
  const request = JSON.parse(line);
  if (request.command === 'end') process.exit(request.code);
  const heldOnArrival = held++;
  const receivedSoFar = ++received;
  setTimeout(() => {
    held--;
    process.stdout.write(JSON.stringify({ id: request.id, ok: true, request, heldOnArrival, received: receivedSoFar }) + '\\n');
  }, request.delay ?? 0);
});
`;

const referenceVehicles: ReferenceVehicleCatalogueEntry[] = [
  {
    name: 'A',
    title: { en: 'Vehicle A', zh: '车辆 A' },
    openArguments: { vehicle_definition_path: '/a.json', startup_state_path: '/a_start.json', contact_record_path: '/a_contact.json', visual_definition_path: '/a_visual.json' },
  },
  {
    name: 'B',
    title: { en: 'Vehicle B', zh: '车辆 B' },
    openArguments: { vehicle_definition_path: '/b.json', startup_state_path: '/b_start.json', contact_record_path: '/b_contact.json', visual_definition_path: null },
  },
];

/** A temporary folder holding the stand-in service, a dist/ with one page, a file beside it, and a folder of projects. */
async function serverFiles() {
  const directory = await mkdtemp(join(tmpdir(), 'orvd-modelling-server-'));
  const servicePath = join(directory, 'fake_service.cjs');
  await writeFile(servicePath, `#!${process.execPath}\n${fakeServiceSource}`);
  await chmod(servicePath, 0o755);
  const staticDirectory = join(directory, 'dist');
  await mkdir(staticDirectory);
  await writeFile(join(staticDirectory, 'index.html'), '<title>modelling</title>');
  await writeFile(join(directory, 'secret.txt'), 'outside dist');
  const vehicleProjectsDirectory = join(directory, 'projects');
  await mkdir(vehicleProjectsDirectory);
  return { directory, options: { servicePath, host: '127.0.0.1', port: 0, staticDirectory, referenceVehicles, vehicleProjectsDirectory, terminal: null } };
}

/** The person at the terminal, answering the questions with the answers given, in turn; what was asked and told is kept. */
function personAnswering(...answers: (string | null)[]) {
  const asked: string[] = [];
  const told: string[] = [];
  const terminal: TerminalConversation = {
    ask: async (question) => {
      asked.push(question);
      return answers.shift() ?? null;
    },
    tell: (line) => {
      told.push(line);
    },
  };
  return { terminal, asked, told };
}

/** How many times the stand-in service has started. */
async function serviceStarts(directory: string): Promise<number> {
  const started = await readFile(join(directory, 'started.txt'), 'utf8').catch(() => '');
  return started.split('\n').filter((line) => line !== '').length;
}

async function fixture() {
  const { directory, options } = await serverFiles();
  const endings: string[] = [];
  const server = await startVehicleModelingHttpServer(options, (ending) => endings.push(ending));
  const origin = `http://127.0.0.1:${server.port}`;
  return { directory, server, origin, endings, vehicleProjectsDirectory: options.vehicleProjectsDirectory };
}

function command(origin: string, body: string, signal?: AbortSignal): Promise<Response> {
  return fetch(`${origin}/api/command`, { method: 'POST', body, signal });
}

function commandFromPage(target: string, pageOrigin: string, id: number): Promise<Response> {
  return fetch(`${target}/api/command`, { method: 'POST', headers: { Origin: pageOrigin }, body: JSON.stringify({ id, command: 'echo' }) });
}

test('commands are relayed one at a time, each answered with its own answer line', { timeout: 10000 }, async () => {
  const { directory, server, origin } = await fixture();
  try {
    // Later requests answer sooner: only strict one-at-a-time relaying keeps every answer with its request.
    const answers = await Promise.all(
      Array.from({ length: 12 }, async (_, id) => (await command(origin, JSON.stringify({ id, command: 'echo', delay: 24 - 2 * id }))).json()),
    );
    answers.forEach((answer, id) => {
      assert.equal(answer.id, id);
      assert.equal(answer.heldOnArrival, 0);
    });
    // A request laid out over several lines still reaches the service as one line.
    const spread = await (await command(origin, '{\n  "id": 99,\n  "command": "echo",\n  "nested": {"a": [1,\n 2]}\n}')).json();
    assert.deepEqual(spread.request, { id: 99, command: 'echo', nested: { a: [1, 2] } });
    assert.equal((await command(origin, 'not json')).status, 400);
  } finally {
    await server.close();
    await rm(directory, { recursive: true, force: true });
  }
});

test('reference vehicles with their open arguments, the folder of projects and the built files are served; nothing else is', { timeout: 10000 }, async () => {
  const { directory, server, origin, vehicleProjectsDirectory } = await fixture();
  try {
    const listed = await (await fetch(`${origin}/api/reference_vehicles`)).json();
    assert.deepEqual(listed, [
      {
        name: 'A',
        title: { en: 'Vehicle A', zh: '车辆 A' },
        open_arguments: { vehicle_definition_path: '/a.json', startup_state_path: '/a_start.json', contact_record_path: '/a_contact.json', visual_definition_path: '/a_visual.json' },
      },
      {
        name: 'B',
        title: { en: 'Vehicle B', zh: '车辆 B' },
        open_arguments: { vehicle_definition_path: '/b.json', startup_state_path: '/b_start.json', contact_record_path: '/b_contact.json', visual_definition_path: null },
      },
    ]);
    assert.deepEqual(await (await fetch(`${origin}/api/vehicle_projects_directory`)).json(), { directory: vehicleProjectsDirectory });
    assert.match(await (await fetch(`${origin}/?vehicle=A`)).text(), /modelling/);
    for (const path of ['/api/reference_vehicles/A/visual_definition', '/api/other', '/%2e%2e/secret.txt', '/missing.js']) {
      assert.equal((await fetch(`${origin}${path}`)).status, 404, path);
    }
  } finally {
    await server.close();
    await rm(directory, { recursive: true, force: true });
  }
});

test('a missing folder of projects is created, parents included, when the person at the terminal answers yes, and the server starts', { timeout: 10000 }, async () => {
  const { directory, options } = await serverFiles();
  try {
    for (const [index, answer] of ['y', 'YES', ' Yes '].entries()) {
      const vehicleProjectsDirectory = join(directory, `absent_${index}`, 'projects');
      const person = personAnswering(answer);
      const server = await startVehicleModelingHttpServer({ ...options, vehicleProjectsDirectory, terminal: person.terminal }, () => {});
      try {
        assert.ok((await stat(vehicleProjectsDirectory)).isDirectory());
        assert.equal(person.asked.length, 1, 'asked once');
        const question = person.asked[0]!;
        assert.ok(question.includes(`'${vehicleProjectsDirectory}'`) && /[一-鿿]/.test(question) && question.endsWith('[y/N] '), question);
        assert.equal(person.told.length, 1);
        assert.ok(person.told[0]!.includes(`'${vehicleProjectsDirectory}'`), 'and says it created the folder');
        assert.deepEqual(await (await fetch(`http://127.0.0.1:${server.port}/api/vehicle_projects_directory`)).json(), { directory: vehicleProjectsDirectory });
      } finally {
        await server.close();
      }
    }
    assert.equal(await serviceStarts(directory), 3);
  } finally {
    await rm(directory, { recursive: true, force: true });
  }
});

test('the server does not start, says why, creates nothing and starts no service for a missing folder answered otherwise, a missing one with nobody to ask, or a file', { timeout: 10000 }, async () => {
  const { directory, options } = await serverFiles();
  const refusal = (path: string) => new RegExp(`the folder of vehicle projects '${path}' is not an existing directory`);
  try {
    const missing = join(directory, 'absent', 'projects');
    for (const answer of ['n', 'no', '', 'yess', null]) {
      const person = personAnswering(answer);
      await assert.rejects(startVehicleModelingHttpServer({ ...options, vehicleProjectsDirectory: missing, terminal: person.terminal }, () => {}), refusal(missing));
      assert.equal(person.asked.length, 1, `asked once, answered ${JSON.stringify(answer)}`);
      assert.deepEqual(person.told, []);
      await assert.rejects(access(join(directory, 'absent')), /ENOENT/, 'nothing is created');
    }
    await assert.rejects(startVehicleModelingHttpServer({ ...options, vehicleProjectsDirectory: missing, terminal: null }, () => {}), refusal(missing));
    await assert.rejects(access(join(directory, 'absent')), /ENOENT/, 'nothing is created');
    const file = join(directory, 'secret.txt');
    for (const person of [null, personAnswering('yes')]) {
      await assert.rejects(startVehicleModelingHttpServer({ ...options, vehicleProjectsDirectory: file, terminal: person?.terminal ?? null }, () => {}), refusal(file));
      assert.deepEqual(person?.asked ?? [], [], 'a file is refused without a question');
    }
    assert.equal(await serviceStarts(directory), 0, 'no service was started');
  } finally {
    await rm(directory, { recursive: true, force: true });
  }
});

test('a service that ends is reported once, with how it ended', { timeout: 10000 }, async () => {
  const { directory, server, origin, endings } = await fixture();
  const unanswered = new AbortController();
  try {
    command(origin, JSON.stringify({ id: 1, command: 'end', code: 3 }), unanswered.signal).catch(() => {});
    for (let wait = 0; wait < 100 && endings.length === 0; ++wait) await new Promise((done) => setTimeout(done, 20));
    assert.deepEqual(endings, ['exited with code 3']);
  } finally {
    unanswered.abort();
    await server.close();
    await rm(directory, { recursive: true, force: true });
  }
});

test('a command from a page of another origin is refused before it reaches the service', { timeout: 10000 }, async () => {
  const { directory, server, origin } = await fixture();
  try {
    assert.equal((await commandFromPage(origin, 'http://elsewhere.invalid', 1)).status, 403);
    assert.equal((await commandFromPage(origin, `http://localhost:${server.port}`, 2)).status, 403);
    assert.equal((await commandFromPage(origin, 'null', 3)).status, 403);
    const own = await (await commandFromPage(origin, origin, 4)).json();
    assert.equal(own.id, 4);
    assert.equal(own.received, 1, 'the refused commands never reached the service');
  } finally {
    await server.close();
    await rm(directory, { recursive: true, force: true });
  }
});

test('pages of the Vite development server reach the service through its proxy', { timeout: 10000 }, async () => {
  const { directory, server, origin } = await fixture();
  // The development configuration's own proxy, pointed at this server.
  const vite = await createViteServer({
    configFile: false,
    logLevel: 'silent',
    server: { host: '127.0.0.1', port: 0, watch: null, proxy: { '/api': { ...modellingServerProxy, target: origin } } },
  });
  try {
    await vite.listen();
    const viteOrigin = `http://127.0.0.1:${(vite.httpServer!.address() as AddressInfo).port}`;
    const answer = await (await commandFromPage(viteOrigin, viteOrigin, 5)).json();
    assert.equal(answer.id, 5);
    assert.equal((await commandFromPage(viteOrigin, 'http://elsewhere.invalid', 6)).status, 403);
  } finally {
    await vite.close();
    await server.close();
    await rm(directory, { recursive: true, force: true });
  }
});
