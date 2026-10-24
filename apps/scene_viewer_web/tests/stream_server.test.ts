import assert from 'node:assert/strict';
import { test } from 'node:test';
import { mkdtemp, mkdir, writeFile, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { createSocket } from 'node:dgram';
import { once } from 'node:events';
import WebSocket from 'ws';
import { createServer as createViteServer } from 'vite';
import { startServer } from '../server/server.ts';
import { StreamClient } from '../server/client.ts';
import type { BinarySender } from '../server/client.ts';
import { parseServerArguments } from '../server/main.ts';
import { StreamReceiver } from '../src/stream/receiver.ts';
import { decodeMessage } from '../src/stream/protocol.ts';
import type { DecodedMessage } from '../src/stream/protocol.ts';
import { contractFixture, fixtureVisual } from './scene_contract_fixtures.ts';
import { datagrams, descriptionPayload, statePayload } from './stream_fixtures.ts';

function update(run = 1n, sequence = 1n) {
  const receiver = new StreamReceiver();
  for (const packet of datagrams(1, descriptionPayload(), run)) receiver.feed(packet);
  receiver.feed(datagrams(2, statePayload(), run, sequence)[0]!);
  return receiver.current!;
}
class SlowSocket implements BinarySender {
  readyState = 1; bufferedAmount = 0;
  sent: DecodedMessage[] = [];
  private callbacks: ((error?: Error) => void)[] = [];
  send(bytes: Uint8Array, callback: (error?: Error) => void): void { this.sent.push(decodeMessage(bytes)); this.callbacks.push(callback); }
  finish(): void { this.callbacks.shift()!(); }
  terminate(): void { this.readyState = 3; }
}
test('slow clients retain only latest state and always receive its run description first', () => {
  const socket = new SlowSocket(); const client = new StreamClient(socket);
  client.publish(update());
  for (let i = 2; i <= 1000; ++i) client.publish(update(1n, BigInt(i)));
  assert.equal(socket.sent.length, 1);
  socket.finish(); assert.equal(socket.sent[1]?.header.sequence, 1000n);
  client.publish(update(2n)); client.publish(update(3n)); socket.finish();
  assert.equal(socket.sent[2]?.kind, 'description'); assert.equal(socket.sent[2]?.header.runId, 3n);
  socket.finish(); assert.equal(socket.sent[3]?.kind, 'state'); assert.equal(socket.sent[3]?.header.runId, 3n);
  socket.finish(); assert.equal(socket.sent.length, 4);
  socket.bufferedAmount = 3 * 1024 * 1024; client.publish(update(3n, 2n)); assert.equal(socket.readyState, 3);
});

test('service arguments expose explicit endpoints and resource mappings', () => {
  const defaults = parseServerArguments([]);
  assert.deepEqual([defaults.host, defaults.port, defaults.udpHost, defaults.udpPort], ['127.0.0.1', 6641, '127.0.0.1', 10099]);
  const args = parseServerArguments(['--port', '8000', '--udp-port', '10100', '--host', '0.0.0.0', '--bundle', 'line=./record']);
  assert.equal(args.port, 8000); assert.equal(args.udpPort, 10100); assert.equal(args.host, '0.0.0.0');
  assert.ok(args.bundles.get('line')?.endsWith('/record'));
  for (const values of [['--port', '0'], ['--port', '3.5'], ['--port', '2', '--port', '3'], ['--bad', 'x'], ['--bundle', 'a=b', '--bundle', 'a=c']]) {
    assert.throws(() => parseServerArguments(values));
  }
});

class Inbox {
  private messages: DecodedMessage[] = [];
  private waiting: ((message: DecodedMessage) => void)[] = [];
  constructor(socket: WebSocket) {
    socket.on('message', (bytes, binary) => {
      assert.equal(binary, true);
      const message = decodeMessage(new Uint8Array(bytes as Buffer));
      const consume = this.waiting.shift(); if (consume) consume(message); else this.messages.push(message);
    });
  }
  next(): Promise<DecodedMessage> { return this.messages.length ? Promise.resolve(this.messages.shift()!) : new Promise((done) => this.waiting.push(done)); }
}

test('real UDP and WebSocket support late joins, reconnect, run replacement and static-only resources', { timeout: 10000 }, async () => {
  const directory = await mkdtemp(join(tmpdir(), 'orvd-stream-'));
  const recordDirectory = join(directory, 'record'); const staticDirectory = join(directory, 'web');
  await mkdir(recordDirectory); await mkdir(staticDirectory);
  const fixture = contractFixture(); fixture.json.assets.visual_definition_file = 'visual_definition.json';
  await writeFile(join(recordDirectory, 'scene.json'), JSON.stringify(fixture.json));
  await writeFile(join(recordDirectory, 'visual_definition.json'), fixtureVisual());
  // There are deliberately no frame files: static resource service must not read them.
  await writeFile(join(staticDirectory, 'index.html'), '<title>ORVD viewer</title>');
  const server = await startServer({ host: '127.0.0.1', port: 0, udpHost: '127.0.0.1', udpPort: 0, staticDirectory, bundles: new Map([['line', recordDirectory]]) });
  const sender = createSocket('udp4'); const sockets: WebSocket[] = [];
  const send = async (packets: Uint8Array[]) => {
    for (const packet of packets) await new Promise<void>((done, reject) => sender.send(packet, server.udpPort, '127.0.0.1', (error) => error ? reject(error) : done()));
  };
  async function connect() {
    const socket = new WebSocket(`ws://127.0.0.1:${server.httpPort}/live`); sockets.push(socket);
    const inbox = new Inbox(socket); await once(socket, 'open'); return { socket, inbox };
  }
  try {
    assert.match(await (await fetch(`http://127.0.0.1:${server.httpPort}/`)).text(), /ORVD/);
    const manifest = await (await fetch(`http://127.0.0.1:${server.httpPort}/bundles/line/scene.json`)).json();
    assert.equal(manifest.schema, 'orvd.scene_bundle'); assert.deepEqual(Object.keys(manifest).sort(), ['assets', 'schema']);
    assert.equal(await (await fetch(`http://127.0.0.1:${server.httpPort}/bundles/line/visual_definition.json`)).text(), fixtureVisual());
    for (const path of ['/bundles/line/frames.f64le', '/bundles/missing/scene.json', '/bundles/line/%2e%2e%2fscene.json']) {
      assert.equal((await fetch(`http://127.0.0.1:${server.httpPort}${path}`)).status, 404);
    }
    const first = await connect();
    await send(datagrams(2, statePayload(), 9007199254741099n, 9007199254741111n));
    await send(datagrams(1, descriptionPayload('line'), 9007199254741099n));
    assert.equal((await first.inbox.next()).kind, 'description');
    const state = await first.inbox.next(); assert.equal(state.header.sequence, 9007199254741111n);
    first.socket.terminate(); await once(first.socket, 'close');
    const rejoined = await connect();
    assert.equal((await rejoined.inbox.next()).kind, 'description'); assert.equal((await rejoined.inbox.next()).header.sequence, state.header.sequence);
    await send(datagrams(1, descriptionPayload('line'), 2n)); await send(datagrams(2, statePayload(), 2n));
    assert.equal((await rejoined.inbox.next()).header.runId, 2n); assert.equal((await rejoined.inbox.next()).header.runId, 2n);
    await send(datagrams(1, descriptionPayload('line'), state.header.runId, 100n));
    await send(datagrams(2, statePayload(), state.header.runId, state.header.sequence + 1n));
    await send(datagrams(2, statePayload(), 2n, 2n));
    const next = await rejoined.inbox.next(); assert.equal(next.kind, 'state'); assert.equal(next.header.runId, 2n); assert.equal(next.header.sequence, 2n);
    const foreign = new WebSocket(`ws://127.0.0.1:${server.httpPort}/live`, { origin: 'http://elsewhere.invalid' });
    foreign.on('error', () => {}); await new Promise<void>((done) => foreign.once('close', () => done()));
  } finally {
    for (const socket of sockets) socket.terminate(); sender.close(); await server.close(); await rm(directory, { recursive: true, force: true });
  }
});

test('a failed UDP bind releases the independently opened HTTP listener', { timeout: 5000 }, async () => {
  const occupied = createSocket('udp4'); occupied.bind(0, '127.0.0.1'); await once(occupied, 'listening');
  try {
    await assert.rejects(startServer({ host: '127.0.0.1', port: 0, udpHost: '127.0.0.1', udpPort: occupied.address().port, staticDirectory: '.', bundles: new Map() }), /EADDRINUSE/);
  } finally { occupied.close(); }
});


test('Vite proxy preserves the browser origin for the live WebSocket', { timeout: 10000 }, async () => {
  const service = await startServer({ host: '127.0.0.1', port: 0, udpHost: '127.0.0.1', udpPort: 0,
    staticDirectory: '.', bundles: new Map() });
  const vite = await createViteServer({ configFile: false, logLevel: 'silent', server: { host: '127.0.0.1', port: 0,
    watch: null, proxy: { '/live': { target: `http://127.0.0.1:${service.httpPort}`, ws: true, changeOrigin: false } } } });
  const sender = createSocket('udp4'); let socket: WebSocket | null = null;
  try {
    await vite.listen(); const address = vite.httpServer!.address();
    assert.ok(address && typeof address !== 'string');
    socket = new WebSocket(`ws://127.0.0.1:${address.port}/live`, { origin: `http://127.0.0.1:${address.port}` });
    const inbox = new Inbox(socket); await once(socket, 'open');
    for (const packet of [...datagrams(1, descriptionPayload()), ...datagrams(2, statePayload())]) {
      await new Promise<void>((done, reject) => sender.send(packet, service.udpPort, '127.0.0.1', (error) => error ? reject(error) : done()));
    }
    assert.equal((await inbox.next()).kind, 'description'); assert.equal((await inbox.next()).kind, 'state');
  } finally { socket?.terminate(); sender.close(); await vite.close(); await service.close(); }
});
