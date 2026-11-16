import { createReadStream } from 'node:fs';
import { readFile, realpath, stat } from 'node:fs/promises';
import { createServer } from 'node:http';
import type { IncomingMessage, ServerResponse } from 'node:http';
import { extname, resolve, sep } from 'node:path';

import type { ReferenceVehicleCatalogueEntry } from './read_reference_vehicle_catalogue.ts';
import { VehicleModelingServiceChildProcess } from './vehicle_modeling_service_child_process.ts';

export interface VehicleModelingHttpServerOptions {
  servicePath: string;
  host: string;
  port: number;
  staticDirectory: string;
  referenceVehicles: ReferenceVehicleCatalogueEntry[];
}

export interface RunningVehicleModelingHttpServer {
  port: number;
  close(): Promise<void>;
}

/** The kinds of file the build writes to dist/. */
const contentTypes: Record<string, string> = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
};

const jsonHeaders = { 'Content-Type': 'application/json; charset=utf-8', 'Cache-Control': 'no-store' };

/**
 * Whether a command may come from where the request says it comes from. A
 * page of another origin must not drive the service through the user's
 * browser, so a request naming its origin must name this server's host, as the
 * scene viewer's server requires of its WebSocket upgrades. The Vite
 * development proxy keeps the browser's Host header, so its pages pass.
 */
function fromOwnOrigin(request: IncomingMessage): boolean {
  const origin = request.headers.origin;
  if (origin === undefined) return true;
  try {
    return new URL(origin).host === request.headers.host;
  } catch {
    return false;
  }
}

async function readBody(request: IncomingMessage): Promise<string> {
  const chunks: Buffer[] = [];
  for await (const chunk of request) {
    chunks.push(chunk as Buffer);
  }
  return Buffer.concat(chunks).toString('utf8');
}

/** Starts the service and the HTTP server; onServiceEnd reports a service that ends on its own. */
export async function startVehicleModelingHttpServer(options: VehicleModelingHttpServerOptions, onServiceEnd: (ending: string) => void): Promise<RunningVehicleModelingHttpServer> {
  const vehicles = new Map(options.referenceVehicles.map((vehicle) => [vehicle.name, vehicle]));
  const referenceVehicleList = JSON.stringify(
    options.referenceVehicles.map((vehicle) => ({
      name: vehicle.name,
      title: vehicle.title,
      open_arguments: vehicle.openArguments,
      visual_definition_available: vehicle.visualDefinitionPath !== null,
    })),
  );
  const service = new VehicleModelingServiceChildProcess(options.servicePath, onServiceEnd);

  async function relayCommand(request: IncomingMessage, response: ServerResponse): Promise<void> {
    let requestLine: string;
    try {
      // Re-encoding makes the request exactly one line, whatever its layout.
      requestLine = JSON.stringify(JSON.parse(await readBody(request)));
    } catch {
      response.writeHead(400, jsonHeaders);
      response.end(JSON.stringify({ message: 'the request body is not JSON' }));
      return;
    }
    const answerLine = await service.exchange(requestLine);
    response.writeHead(200, jsonHeaders);
    response.end(answerLine);
  }

  async function respond(request: IncomingMessage, response: ServerResponse): Promise<void> {
    const pathname = new URL(request.url ?? '/', 'http://localhost').pathname;
    if (pathname === '/api/command' && request.method === 'POST') {
      if (fromOwnOrigin(request)) {
        await relayCommand(request, response);
      } else {
        response.writeHead(403, jsonHeaders);
        response.end(JSON.stringify({ message: 'commands are taken only from pages of this server' }));
      }
      return;
    }
    if (request.method !== 'GET' && request.method !== 'HEAD') {
      response.writeHead(405);
      response.end();
      return;
    }
    if (pathname === '/api/reference_vehicles') {
      response.writeHead(200, jsonHeaders);
      response.end(request.method === 'HEAD' ? undefined : referenceVehicleList);
      return;
    }
    const visual = /^\/api\/reference_vehicles\/([^/]+)\/visual_definition$/.exec(pathname);
    if (visual !== null) {
      const path = vehicles.get(decodeURIComponent(visual[1]!))?.visualDefinitionPath;
      if (path === undefined || path === null) {
        throw new Error('no visual definition for this name');
      }
      const text = await readFile(path, 'utf8');
      response.writeHead(200, jsonHeaders);
      response.end(request.method === 'HEAD' ? undefined : text);
      return;
    }
    if (pathname.startsWith('/api/')) {
      throw new Error('unknown API path');
    }
    const root = await realpath(options.staticDirectory);
    const path = await realpath(resolve(root, `.${decodeURIComponent(pathname === '/' ? '/index.html' : pathname)}`));
    if (!path.startsWith(root + sep) || !(await stat(path)).isFile()) {
      throw new Error('unknown static file');
    }
    response.writeHead(200, { 'Content-Type': contentTypes[extname(path)] ?? 'application/octet-stream' });
    if (request.method === 'HEAD') response.end();
    else createReadStream(path).on('error', () => response.destroy()).pipe(response);
  }

  const http = createServer((request, response) => {
    respond(request, response).catch(() => {
      if (!response.headersSent) response.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
      response.end('Not found');
    });
  });
  try {
    await new Promise<void>((accept, reject) => {
      http.once('error', reject);
      http.listen(options.port, options.host, () => {
        http.off('error', reject);
        accept();
      });
    });
  } catch (error) {
    await service.stop();
    throw error;
  }
  const address = http.address();
  if (address === null || typeof address === 'string') {
    throw new Error('the HTTP listener has no TCP address');
  }
  return {
    port: address.port,
    close: async () => {
      http.closeAllConnections();
      await new Promise<void>((done) => http.close(() => done()));
      await service.stop();
    },
  };
}
