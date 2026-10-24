import { fileURLToPath } from 'node:url';
import { resolve } from 'node:path';
import { startServer } from './server.ts';
import type { ServerOptions } from './server.ts';

export function parseServerArguments(args: string[]): ServerOptions {
  const options: ServerOptions = { host: '127.0.0.1', port: 6641, udpHost: '127.0.0.1', udpPort: 10099,
    staticDirectory: fileURLToPath(new URL('../dist', import.meta.url)), bundles: new Map() };
  const bundles = new Map<string, string>();
  const seen = new Set<string>();
  for (let i = 0; i < args.length; i += 2) {
    const key = args[i]!;
    const value = args[i + 1];
    if (!value || value.startsWith('--')) throw new Error(`Missing value for ${key}`);
    if (key !== '--bundle' && seen.has(key)) throw new Error(`Repeated option ${key}`);
    seen.add(key);
    if (key === '--bundle') {
      const separator = value.indexOf('=');
      if (separator < 1 || separator === value.length - 1) throw new Error('--bundle requires NAME=DIRECTORY');
      const name = value.slice(0, separator);
      if (bundles.has(name)) throw new Error(`Repeated resource name ${name}`);
      bundles.set(name, resolve(value.slice(separator + 1)));
    } else if (key === '--host') options.host = value;
    else if (key === '--udp-host') options.udpHost = value;
    else if (key === '--port' || key === '--udp-port') {
      if (!/^\d+$/.test(value) || Number(value) < 1 || Number(value) > 65535) throw new Error(`${key} requires a port from 1 to 65535`);
      if (key === '--port') options.port = Number(value); else options.udpPort = Number(value);
    } else throw new Error(`Unknown option ${key}`);
  }
  return { ...options, bundles };
}
if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  try {
    const options = parseServerArguments(process.argv.slice(2));
    const server = await startServer(options);
    process.stdout.write(`Live viewer: http://${options.host}:${server.httpPort}/?live=1\nUDP: ${options.udpHost}:${server.udpPort}\n`);
    let stopping = false;
    const stop = () => { if (!stopping) { stopping = true; void server.close(); } };
    process.on('SIGINT', stop);
    process.on('SIGTERM', stop);
  } catch (error) {
    process.stderr.write(`${error instanceof Error ? error.message : error}\n`);
    process.exitCode = 1;
  }
}
