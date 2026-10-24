import * as field from './json_fields.ts';
import { parseSceneAssets, type SceneAssets } from './scene_assets.ts';

export class ResourceReadError extends Error {}

async function readResource<T>(url: string, signal: AbortSignal, read: (response: Response) => Promise<T>): Promise<T> {
  try {
    const response = await fetch(url, { signal });
    if (!response.ok) throw new Error(`could not fetch ${url}: ${response.status}`);
    return await read(response);
  } catch (error) {
    if (signal.aborted) throw error;
    throw new ResourceReadError(error instanceof Error ? error.message : String(error), { cause: error });
  }
}
export function fetchBytes(url: string, signal: AbortSignal): Promise<ArrayBuffer> {
  return readResource(url, signal, (response) => response.arrayBuffer());
}
export function fetchText(url: string, signal: AbortSignal): Promise<string> {
  return readResource(url, signal, (response) => response.text());
}

/** Resolve each directory independently against the page, never against another resource directory. */
export function manifestUrl(url: string, fileName: string): URL {
  const resolved = new URL(url, globalThis.location?.href);
  if (!resolved.pathname.endsWith(`/${fileName}`)) {
    resolved.pathname = `${resolved.pathname.replace(/\/$/, '')}/${fileName}`;
  }
  return resolved;
}

export function parseSceneBundle(value: unknown, visualDefinitionText: string | null = null): SceneAssets {
  const bundle = field.object(value, 'bundle');
  field.exact(bundle['schema'], 'orvd.scene_bundle', 'bundle.schema');
  return parseSceneAssets(bundle['assets'], visualDefinitionText);
}

export async function loadBundleFromUrl(url: string, signal: AbortSignal): Promise<SceneAssets> {
  const manifest = manifestUrl(url, 'scene.json');
  const assets = parseSceneBundle(JSON.parse(await fetchText(manifest.href, signal)));
  const visualDefinitionText = assets.visualDefinitionFile === null ? null :
    await fetchText(new URL(assets.visualDefinitionFile, manifest).href, signal);
  return { ...assets, visualDefinitionText };
}
