import { createSocket } from 'node:dgram';
import { createServer } from 'node:http';
import type { IncomingMessage, ServerResponse } from 'node:http';
import { readFile, realpath, stat } from 'node:fs/promises';
import { createReadStream } from 'node:fs';
import { resolve, sep, extname } from 'node:path';
import { WebSocketServer } from 'ws';
import { StreamReceiver } from '../src/stream/receiver.ts';
import { StreamClient } from './client.ts';

export interface ServerOptions {
  host: string;
  port: number;
  udpHost: string;
  udpPort: number;
  staticDirectory: string;
  bundles: ReadonlyMap<string, string>;
}
export interface RunningServer { httpPort: number; udpPort: number; close(): Promise<void>; }
interface Assets { manifest: string; visual: string | null; }
const contentTypes: Record<string, string> = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8', '.json': 'application/json; charset=utf-8', '.svg': 'image/svg+xml', '.png': 'image/png', '.ico': 'image/x-icon', '.woff2': 'font/woff2' };

export async function startServer(options: ServerOptions): Promise<RunningServer> {
  const receiver = new StreamReceiver();
  const clients = new Set<StreamClient>();
  const resourceCache = new Map<string, Promise<Assets>>();
  async function resources(name: string): Promise<Assets> {
    const directory = options.bundles.get(name);
    if (!directory) throw new Error('Unknown resource bundle');
    let cached = resourceCache.get(name);
    if (!cached) {
      cached = (async () => {
        const envelope = JSON.parse(await readFile(resolve(directory, 'scene.json'), 'utf8')) as Record<string, unknown>;
        if (envelope['schema'] !== 'orvd.scene_record' && envelope['schema'] !== 'orvd.scene_bundle') throw new Error('Unsupported resource schema');
        const assets = envelope['assets'];
        if (!assets || typeof assets !== 'object' || Array.isArray(assets)) throw new Error('Missing scene assets');
        const visualFile = (assets as Record<string, unknown>)['visual_definition_file'];
        if (visualFile !== null && visualFile !== 'visual_definition.json') throw new Error('Invalid visual resource name');
        const visual = visualFile === null ? null : await readFile(resolve(directory, 'visual_definition.json'), 'utf8');
        return { manifest: JSON.stringify({ schema: 'orvd.scene_bundle', assets }), visual };
      })();
      resourceCache.set(name, cached);
      cached.catch(() => resourceCache.delete(name));
    }
    return cached;
  }
  async function respond(request: IncomingMessage, response: ServerResponse): Promise<void> {
    if (request.method !== 'GET' && request.method !== 'HEAD') { response.writeHead(405); response.end(); return; }
    const pathname = new URL(request.url ?? '/', 'http://localhost').pathname;
    const bundle = /^\/bundles\/([^/]+)\/(scene\.json|visual_definition\.json)$/.exec(pathname);
    if (bundle) {
      const assets = await resources(decodeURIComponent(bundle[1]!));
      const text = bundle[2] === 'scene.json' ? assets.manifest : assets.visual;
      if (text === null) throw new Error('Missing visual resource');
      response.writeHead(200, { 'Content-Type': 'application/json; charset=utf-8', 'Cache-Control': 'no-cache' });
      response.end(request.method === 'HEAD' ? undefined : text);
      return;
    }
    if (pathname.startsWith('/bundles/') || pathname === '/live') throw new Error('Unknown resource');
    const root = await realpath(options.staticDirectory);
    const path = await realpath(resolve(root, `.${decodeURIComponent(pathname === '/' ? '/index.html' : pathname)}`));
    if (!path.startsWith(root + sep) || !(await stat(path)).isFile()) throw new Error('Unknown static file');
    response.writeHead(200, { 'Content-Type': contentTypes[extname(path)] ?? 'application/octet-stream' });
    if (request.method === 'HEAD') response.end();
    else createReadStream(path).on('error', () => response.destroy()).pipe(response);
  }
  const http = createServer((request, response) => {
    void respond(request, response).catch(() => {
      if (!response.headersSent) response.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
      response.end('Resource not found');
    });
  });
  const websocket = new WebSocketServer({ noServer: true, maxPayload: 1024, perMessageDeflate: false });
  http.on('upgrade', (request, socket, head) => {
    const origin = request.headers.origin;
    let allowed = false;
    try {
      allowed = new URL(request.url ?? '/', 'http://localhost').pathname === '/live';
      if (origin) allowed &&= new URL(origin).host === request.headers.host;
    } catch { allowed = false; }
    if (!allowed) { socket.destroy(); return; }
    websocket.handleUpgrade(request, socket, head, (client) => websocket.emit('connection', client));
  });
  websocket.on('connection', (socket) => {
    const client = new StreamClient(socket);
    clients.add(client);
    socket.on('close', () => clients.delete(client));
    socket.on('error', () => socket.terminate());
    // This channel is observation-only; clients do not send commands.
    socket.on('message', () => socket.close(1008, 'Observation-only connection'));
    const current = receiver.current;
    if (current) client.publish(current);
  });
  const udp = createSocket(options.udpHost.includes(':') ? 'udp6' : 'udp4');
  udp.on('message', (packet) => {
    try {
      const update = receiver.feed(packet);
      if (update) for (const client of clients) client.publish(update);
    } catch { /* An invalid datagram does not terminate the receiving channel. */ }
  });
  const listenHttp = new Promise<void>((accept, reject) => {
    http.once('error', reject);
    http.listen(options.port, options.host, () => { http.off('error', reject); accept(); });
  });
  const listenUdp = new Promise<void>((accept, reject) => {
    udp.once('error', reject);
    udp.bind(options.udpPort, options.udpHost, () => { udp.off('error', reject); accept(); });
  });
  const started = await Promise.allSettled([listenHttp, listenUdp]);
  const failed = started.find((result) => result.status === 'rejected');
  if (failed?.status === 'rejected') {
    await Promise.all([new Promise<void>((done) => http.close(() => done())),
      new Promise<void>((done) => {
        try { udp.close(() => done()); } catch { done(); }
      }), new Promise<void>((done) => websocket.close(() => done()))]);
    throw failed.reason;
  }
  // Datagram errors after binding are reported without discarding the active display.
  udp.on('error', (error) => process.stderr.write(`UDP receive: ${error.message}\n`));
  const address = http.address();
  if (!address || typeof address === 'string') throw new Error('HTTP listener has no TCP address');
  return { httpPort: address.port, udpPort: udp.address().port, close: async () => {
    for (const socket of websocket.clients) socket.terminate();
    http.closeAllConnections();
    await Promise.all([new Promise<void>((done) => websocket.close(() => done())),
      new Promise<void>((done) => http.close(() => done())), new Promise<void>((done) => udp.close(() => done()))]);
  } };
}
