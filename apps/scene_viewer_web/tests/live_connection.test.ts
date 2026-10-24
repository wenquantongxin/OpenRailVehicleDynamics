import assert from 'node:assert/strict';
import { test } from 'node:test';
import { setImmediate } from 'node:timers/promises';
import { LiveConnection, type LiveConnectionStatus } from '../src/live/live_connection.ts';
import { contractFixture } from './scene_contract_fixtures.ts';

class Socket {
  binaryType = '';
  onopen: (() => void) | null = null;
  onmessage: ((event: MessageEvent) => void) | null = null;
  onclose: (() => void) | null = null;
  onerror: (() => void) | null = null;
  close() { this.onclose?.(); }
  send(bytes: ArrayBuffer) { this.onmessage?.({ data: bytes } as MessageEvent); }
}

function wire(kind: 1 | 2, runId: bigint, sequence: bigint, time: number, payload: Uint8Array): ArrayBuffer {
  const buffer = new ArrayBuffer(40 + payload.byteLength);
  const view = new DataView(buffer);
  view.setUint32(0, 0x5354534f, true);
  view.setUint16(4, 1, true);
  view.setUint16(6, kind, true);
  view.setBigUint64(8, runId, true);
  view.setBigUint64(16, sequence, true);
  view.setFloat64(24, time, true);
  view.setUint16(34, Math.max(1, Math.ceil(payload.byteLength / 1160)), true);
  view.setUint32(36, payload.byteLength, true);
  new Uint8Array(buffer, 40).set(payload);
  return buffer;
}
function description(runId: bigint, bundle: string | null = null, name = 'wheel'): ArrayBuffer {
  const json = contractFixture().json.description;
  json.bodies[0]!.name = name;
  json.wheel_placements[0]!.wheel_body_name = name;
  json.wheel_placements[0]!.carrier_body_name = name;
  return wire(1, runId, 0n, 0, new TextEncoder().encode(JSON.stringify({ schema: 'orvd.scene_description', description: json, resource_bundle: bundle })));
}
function state(runId: bigint, sequence: bigint, time: number): ArrayBuffer {
  const body = new Uint8Array(121);
  const view = new DataView(body.buffer);
  view.setUint32(0, 1, true);
  view.setFloat64(4 + 3 * 8, 1, true);
  view.setUint32(108, 1, true);
  body[120] = 1;
  return wire(2, runId, sequence, time, body);
}

test('live installation waits for a description and state; repeated descriptions do not reinstall', async () => {
  const socket = new Socket();
  const ready: bigint[] = [];
  const sequences: bigint[] = [];
  const connection = new LiveConnection({ ready: (_, d) => ready.push(d.header.runId), state: (s) => sequences.push(s.header.sequence), status: () => {}, disconnected: () => {} },
    null, 'http://localhost:6641/?live=1', (url) => { assert.equal(url, 'ws://localhost:6641/live'); return socket as unknown as WebSocket; });
  connection.start();
  socket.onopen?.();
  socket.send(state(7n, 1n, 1));
  assert.deepEqual(ready, []);
  socket.send(description(7n));
  await setImmediate();
  assert.deepEqual(ready, []);
  socket.send(state(7n, 2n, 2));
  assert.deepEqual(ready, [7n]);
  socket.send(description(7n));
  socket.send(state(7n, 3n, 3));
  assert.deepEqual(ready, [7n]);
  assert.deepEqual(sequences, [3n]);
  connection.dispose();
});

test('late resources cannot overwrite a newer run and resource failure is not silently replaced', async (t) => {
  const oldLocation = Object.getOwnPropertyDescriptor(globalThis, 'location');
  Object.defineProperty(globalThis, 'location', { configurable: true, value: new URL('http://localhost:6641/') });
  t.after(() => { if (oldLocation) Object.defineProperty(globalThis, 'location', oldLocation); else Reflect.deleteProperty(globalThis, 'location'); });
  const assets = contractFixture().json.assets;
  let completeOld: ((response: Response) => void) | undefined;
  t.mock.method(globalThis, 'fetch', async (url: string) => {
    if (url.includes('/old/')) return new Promise<Response>((resolve) => { completeOld = resolve; });
    if (url.includes('/missing/')) return new Response('', { status: 404 });
    return new Response(JSON.stringify({ schema: 'orvd.scene_bundle', assets }));
  });
  const socket = new Socket();
  const ready: bigint[] = [];
  const statuses: LiveConnectionStatus[] = [];
  const connection = new LiveConnection({ ready: (_, d) => ready.push(d.header.runId), state: () => {}, status: (s) => statuses.push(s), disconnected: () => {} },
    '/selected/', 'http://localhost:6641/', () => socket as unknown as WebSocket);
  connection.start();
  socket.send(description(1n, 'old'));
  socket.send(state(1n, 1n, 1));
  socket.send(description(2n));
  socket.send(state(2n, 1n, 2));
  await setImmediate();
  assert.deepEqual(ready, [2n]);
  completeOld?.(new Response(JSON.stringify({ schema: 'orvd.scene_bundle', assets })));
  await setImmediate();
  assert.deepEqual(ready, [2n]);
  socket.send(description(3n, 'missing'));
  socket.send(state(3n, 1n, 3));
  await setImmediate();
  assert.deepEqual(ready, [2n]);
  assert.equal(statuses.at(-1)?.state, 'error');
  assert.match(statuses.at(-1)!.message, /404/);
  connection.dispose();
});

test('backwards time before installation blocks that run across reconnect; a new run clears it', async (t) => {
  t.mock.timers.enable({ apis: ['setTimeout'] });
  const sockets: Socket[] = [];
  const ready: bigint[] = [];
  const statuses: LiveConnectionStatus[] = [];
  const connection = new LiveConnection({ ready: (_, d) => ready.push(d.header.runId), state: () => {}, status: (s) => statuses.push(s), disconnected: () => {} },
    null, 'http://localhost:6641/', () => { const socket = new Socket(); sockets.push(socket); return socket as unknown as WebSocket; });
  connection.start();
  const first = sockets[0]!;
  first.send(description(9n));
  first.send(state(9n, 1n, 2));
  first.send(state(9n, 2n, 1));
  await setImmediate();
  assert.equal(statuses.at(-1)?.state, 'error');
  assert.deepEqual(ready, []);
  first.close();
  t.mock.timers.tick(250);
  const second = sockets[1]!;
  second.onopen?.();
  second.send(description(9n));
  second.send(state(9n, 3n, 3));
  await setImmediate();
  assert.equal(statuses.at(-1)?.state, 'error');
  assert.deepEqual(ready, []);
  second.send(description(10n));
  second.send(state(10n, 1n, 0));
  await setImmediate();
  assert.deepEqual(ready, [10n]);
  assert.equal(statuses.at(-1)?.state, 'connected');
  connection.dispose();
});

test('same-run changed description is rejected and closed connections stop reconnecting after disposal', async (t) => {
  t.mock.timers.enable({ apis: ['setTimeout'] });
  const sockets: Socket[] = [];
  const statuses: LiveConnectionStatus[] = [];
  const connection = new LiveConnection({ ready: () => {}, state: () => {}, status: (s) => statuses.push(s), disconnected: () => {} },
    null, 'https://example.test/', (url) => { assert.equal(url, 'wss://example.test/live'); const socket = new Socket(); sockets.push(socket); return socket as unknown as WebSocket; });
  connection.start();
  const first = sockets[0]!;
  first.send(description(1n));
  first.send(state(1n, 1n, 0));
  await setImmediate();
  first.send(description(1n, null, 'changed'));
  assert.equal(statuses.at(-1)?.state, 'error');
  connection.dispose();
  t.mock.timers.tick(20000);
  assert.equal(sockets.length, 1);
});

test('reconnect keeps the run watermark and permits its exact cached last state', async (t) => {
  t.mock.timers.enable({ apis: ['setTimeout'] });
  const sockets: Socket[] = [];
  const ready: bigint[] = [];
  const statuses: LiveConnectionStatus[] = [];
  const connection = new LiveConnection({ ready: (_, _d, s) => ready.push(s.header.sequence), state: () => {}, status: (s) => statuses.push(s), disconnected: () => {} },
    null, 'http://localhost:6641/', () => { const socket = new Socket(); sockets.push(socket); return socket as unknown as WebSocket; });
  connection.start();
  sockets[0]!.send(description(7n));
  sockets[0]!.send(state(7n, 50n, 5));
  await setImmediate();
  sockets[0]!.close();
  t.mock.timers.tick(250);
  sockets[1]!.send(description(7n));
  sockets[1]!.send(state(7n, 49n, 4));
  await setImmediate();
  assert.deepEqual(ready, [50n]);
  sockets[1]!.send(state(7n, 50n, 5));
  assert.deepEqual(ready, [50n, 50n]);
  sockets[1]!.send(state(7n, 51n, 4));
  assert.equal(statuses.at(-1)?.state, 'error');
  connection.dispose();
});


test('open-close cycles back off until a complete scene resumes, then reset once', async (t) => {
  t.mock.timers.enable({ apis: ['setTimeout'] });
  const sockets: Socket[] = [];
  const statuses: LiveConnectionStatus[] = [];
  const connection = new LiveConnection({ ready: () => {}, state: () => {}, status: (s) => statuses.push(s), disconnected: () => {} },
    null, 'http://localhost:6641/', () => { const socket = new Socket(); sockets.push(socket); return socket as unknown as WebSocket; });
  t.after(() => connection.dispose());
  connection.start();
  for (const delay of [250, 500, 1000, 2000, 4000, 5000, 5000]) {
    const count = sockets.length;
    const socket = sockets.at(-1)!;
    socket.onopen?.();
    assert.notEqual(statuses.at(-1)?.state, 'connected');
    socket.close();
    socket.close(); // A second close notification must not add a second retry.
    t.mock.timers.tick(delay - 1);
    assert.equal(sockets.length, count);
    t.mock.timers.tick(1);
    assert.equal(sockets.length, count + 1);
  }
  const last = sockets.at(-1)!;
  last.send(description(1n));
  await setImmediate();
  assert.notEqual(statuses.at(-1)?.state, 'connected');
  last.send(state(1n, 1n, 1));
  assert.equal(statuses.at(-1)?.state, 'connected');
  const count = sockets.length;
  last.close();
  t.mock.timers.tick(249);
  assert.equal(sockets.length, count);
  t.mock.timers.tick(1);
  assert.equal(sockets.length, count + 1);
  sockets.at(-1)!.close();
  connection.dispose();
  t.mock.timers.tick(20000);
  assert.equal(sockets.length, count + 1);
});

for (const failure of ['network', 'http', 'body'] as const) {
  test(`${failure} resource read failure retries the same run and clears the error only when ready`, async (t) => {
    t.mock.timers.enable({ apis: ['setTimeout'] });
    let requests = 0;
    t.mock.method(globalThis, 'fetch', async () => {
      ++requests;
      if (requests === 1) {
        if (failure === 'network') throw new TypeError('offline');
        if (failure === 'http') return new Response('', { status: 503 });
        const response = new Response('');
        t.mock.method(response, 'text', async () => { throw new Error('body interrupted'); });
        return response;
      }
      return new Response(JSON.stringify({ schema: 'orvd.scene_bundle', assets: contractFixture().json.assets }));
    });
    const sockets: Socket[] = [];
    const ready: bigint[] = [];
    const statuses: LiveConnectionStatus[] = [];
    const connection = new LiveConnection({ ready: (_, d) => ready.push(d.header.runId), state: () => {}, status: (s) => statuses.push(s), disconnected: () => {} },
      'http://localhost:6641/bundle/', 'http://localhost:6641/', () => { const socket = new Socket(); sockets.push(socket); return socket as unknown as WebSocket; });
    t.after(() => connection.dispose());
    connection.start();
    sockets[0]!.send(description(11n));
    sockets[0]!.send(state(11n, 1n, 1));
    await setImmediate();
    assert.equal(statuses.at(-1)?.state, 'error');
    assert.deepEqual(ready, []);
    t.mock.timers.tick(250);
    const recovered = sockets[1]!;
    recovered.onopen?.();
    recovered.send(description(11n));
    await setImmediate();
    assert.equal(statuses.at(-1)?.state, 'error');
    recovered.send(state(11n, 1n, 1));
    assert.deepEqual(ready, [11n]);
    assert.equal(requests, 2);
    assert.deepEqual(statuses.at(-1), { state: 'connected', message: '' });
  });
}

for (const content of ['json', 'schema', 'coordinates', 'visual'] as const) {
  test(`${content} content errors remain errors instead of retrying resource transport`, async (t) => {
    t.mock.timers.enable({ apis: ['setTimeout'] });
    const assets = contractFixture().json.assets;
    if (content === 'coordinates') assets.world_frame = 'different';
    if (content === 'visual') assets.visual_definition_file = 'visual_definition.json';
    let requests = 0;
    t.mock.method(globalThis, 'fetch', async (url: string) => {
      requests++;
      if (url.endsWith('visual_definition.json')) return new Response('{}');
      return new Response(content === 'json' ? '{' : JSON.stringify({ schema: content === 'schema' ? 'wrong' : 'orvd.scene_bundle', assets }));
    });
    const sockets: Socket[] = [];
    const ready: bigint[] = [];
    const statuses: LiveConnectionStatus[] = [];
    const connection = new LiveConnection({ ready: (_, d) => ready.push(d.header.runId), state: () => {}, status: (s) => statuses.push(s), disconnected: () => {} },
      'http://localhost:6641/bundle/', 'http://localhost:6641/', () => { const socket = new Socket(); sockets.push(socket); return socket as unknown as WebSocket; });
    t.after(() => connection.dispose());
    connection.start();
    sockets[0]!.send(description(1n));
    sockets[0]!.send(state(1n, 1n, 1));
    await setImmediate();
    assert.equal(statuses.at(-1)?.state, 'error');
    const failedRequests = requests;
    t.mock.timers.tick(20000);
    assert.equal(sockets.length, 1);
    sockets[0]!.close();
    t.mock.timers.tick(250);
    sockets[1]!.send(description(1n));
    sockets[1]!.send(state(1n, 2n, 2));
    await setImmediate();
    assert.equal(requests, failedRequests);
    assert.deepEqual(ready, []);
    assert.equal(statuses.at(-1)?.state, 'error');
  });
}

test('same-run reconnect reuses loaded resources, while a new run loads and publishes its own definition', async (t) => {
  t.mock.timers.enable({ apis: ['setTimeout'] });
  let requests = 0;
  t.mock.method(globalThis, 'fetch', async () => {
    requests++;
    return new Response(JSON.stringify({ schema: 'orvd.scene_bundle', assets: contractFixture().json.assets }));
  });
  const sockets: Socket[] = [];
  const definitions: unknown[] = [];
  const sequences: bigint[] = [];
  const connection = new LiveConnection({ ready: (d, _, s) => { definitions.push(d); sequences.push(s.header.sequence); }, state: () => {}, status: () => {}, disconnected: () => {} },
    'http://localhost:6641/bundle/', 'http://localhost:6641/', () => { const socket = new Socket(); sockets.push(socket); return socket as unknown as WebSocket; });
  t.after(() => connection.dispose());
  connection.start();
  sockets[0]!.send(description(1n));
  sockets[0]!.send(state(1n, 50n, 5));
  await setImmediate();
  assert.equal(requests, 1);
  sockets[0]!.close();
  t.mock.timers.tick(250);
  const second = sockets[1]!;
  second.send(description(1n));
  second.send(state(1n, 49n, 4));
  assert.equal(definitions.length, 1);
  second.send(state(1n, 50n, 5));
  assert.deepEqual(sequences, [50n, 50n]);
  assert.equal(definitions[0], definitions[1]);
  assert.equal(requests, 1);
  second.send(description(2n));
  second.send(state(2n, 1n, 0));
  await setImmediate();
  assert.equal(requests, 2);
  assert.equal(definitions.length, 3);
  assert.notEqual(definitions[2], definitions[0]);
});

test('an obsolete failed resource request cannot close a newer run connection', async (t) => {
  let rejectOld: ((error: Error) => void) | undefined;
  let requests = 0;
  t.mock.method(globalThis, 'fetch', async () => {
    if (++requests === 1) return new Promise<Response>((_, reject) => { rejectOld = reject; });
    return new Response(JSON.stringify({ schema: 'orvd.scene_bundle', assets: contractFixture().json.assets }));
  });
  const socket = new Socket();
  const close = t.mock.method(socket, 'close');
  const ready: bigint[] = [];
  const statuses: LiveConnectionStatus[] = [];
  const connection = new LiveConnection({ ready: (_, d) => ready.push(d.header.runId), state: () => {}, status: (s) => statuses.push(s), disconnected: () => {} },
    'http://localhost:6641/bundle/', 'http://localhost:6641/', () => socket as unknown as WebSocket);
  t.after(() => connection.dispose());
  connection.start();
  socket.send(description(1n));
  socket.send(state(1n, 1n, 1));
  socket.send(description(2n));
  socket.send(state(2n, 1n, 2));
  await setImmediate();
  assert.deepEqual(ready, [2n]);
  rejectOld?.(new TypeError('late failure'));
  await setImmediate();
  assert.equal(close.mock.callCount(), 0);
  assert.deepEqual(ready, [2n]);
  assert.equal(statuses.at(-1)?.state, 'connected');
});
