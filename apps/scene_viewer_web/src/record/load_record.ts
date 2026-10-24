import { parseRecordEnvelope, parseSceneRecord, type SceneRecord } from './scene_record.ts';
import { parseSceneDefinition } from './scene_definition.ts';
import { parseSceneAssets } from './scene_assets.ts';
import { fetchBytes, fetchText, loadBundleFromUrl, manifestUrl } from './load_bundle.ts';
import * as field from './json_fields.ts';

/** A selected bundle supplies resources; relative record and bundle URLs are page-relative. */
export async function loadRecordFromUrl(baseUrl: string, signal: AbortSignal, bundleUrl: string | null = null): Promise<SceneRecord> {
  const manifest = manifestUrl(baseUrl, 'scene.json');
  const sceneText = await fetchText(manifest.href, signal);
  const scene = parseRecordEnvelope(JSON.parse(sceneText));
  parseSceneDefinition(scene['description']);
  const assets = parseSceneAssets(scene['assets']);
  const frameTable = field.object(scene['frame_table'], 'frame_table');
  const statusTable = field.object(scene['scalar_status_table'], 'scalar_status_table');
  // Resolve the bundle before fetching the selected visual. An overridden record
  // visual need not be present, while a missing bundle item keeps the record item.
  const bundle = bundleUrl === null ? null : await loadBundleFromUrl(bundleUrl, signal);
  const [frames, statuses, visualText] = await Promise.all([
    fetchBytes(new URL(field.string(frameTable['file'], 'frame file'), manifest).href, signal),
    fetchBytes(new URL(field.string(statusTable['file'], 'status file'), manifest).href, signal),
    assets.visualDefinitionFile !== null && (bundle === null || bundle.visualDefinitionFile === null)
      ? fetchText(new URL(assets.visualDefinitionFile, manifest).href, signal) : Promise.resolve(null),
  ]);
  return parseSceneRecord(scene, frames, statuses, visualText, bundle);
}

/** Folder selection is self-contained; it never loads a URL-selected bundle. */
export async function loadRecordFromFiles(files: FileList): Promise<SceneRecord> {
  const byName = new Map<string, File>();
  for (const file of Array.from(files)) {
    if (byName.has(file.name)) throw new Error(`the chosen folder contains more than one ${file.name}`);
    byName.set(file.name, file);
  }
  const sceneFile = byName.get('scene.json');
  if (sceneFile === undefined) throw new Error('the chosen folder has no scene.json; choose a record folder');
  const sceneText = await sceneFile.text();
  const scene = parseRecordEnvelope(JSON.parse(sceneText));
  parseSceneDefinition(scene['description']);
  const assets = parseSceneAssets(scene['assets']);
  const frameTable = field.object(scene['frame_table'], 'frame_table');
  const statusTable = field.object(scene['scalar_status_table'], 'scalar_status_table');
  const framesFile = byName.get(field.string(frameTable['file'], 'frame file'));
  const statusesFile = byName.get(field.string(statusTable['file'], 'status file'));
  if (framesFile === undefined || statusesFile === undefined) throw new Error('the chosen folder lacks the frame or status table named in scene.json');
  const visualText = assets.visualDefinitionFile === null ? null : await (byName.get(assets.visualDefinitionFile)?.text() ?? Promise.resolve(null));
  if (assets.visualDefinitionFile !== null && visualText === null) throw new Error(`the chosen folder lacks ${assets.visualDefinitionFile}`);
  const [frames, statuses] = await Promise.all([framesFile.arrayBuffer(), statusesFile.arrayBuffer()]);
  return parseSceneRecord(scene, frames, statuses, visualText);
}
