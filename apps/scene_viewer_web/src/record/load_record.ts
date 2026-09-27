import { parseSceneRecord, type SceneRecord } from './scene_record.ts';

// Two ways to bring a record into the viewer: fetching a served directory
// (`?record=/records/name/`) or picking the directory from disk. Both end in
// the same parser; nothing else in the viewer knows where the bytes came from.

const sceneFileName = 'scene.json';

async function fetchBytes(url: string): Promise<ArrayBuffer> {
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error(`could not fetch ${url}: ${response.status}`);
  }
  return response.arrayBuffer();
}

async function fetchText(url: string): Promise<string> {
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error(`could not fetch ${url}: ${response.status}`);
  }
  return response.text();
}

export async function loadRecordFromUrl(baseUrl: string): Promise<SceneRecord> {
  const base = baseUrl.endsWith('/') ? baseUrl : `${baseUrl}/`;
  const sceneText = await fetchText(`${base}${sceneFileName}`);
  const scene = JSON.parse(sceneText) as Record<string, unknown>;
  const frameTable = scene['frame_table'] as Record<string, unknown>;
  const statusTable = scene['scalar_status_table'] as Record<string, unknown>;
  const visualFile = scene['visual_definition_file'];
  const [frames, statuses, visualText] = await Promise.all([
    fetchBytes(`${base}${String(frameTable['file'])}`),
    fetchBytes(`${base}${String(statusTable['file'])}`),
    typeof visualFile === 'string' ? fetchText(`${base}${visualFile}`) : Promise.resolve(null),
  ]);
  return parseSceneRecord(sceneText, frames, statuses, visualText);
}

export async function loadRecordFromFiles(files: FileList): Promise<SceneRecord> {
  const byName = new Map<string, File>();
  for (const file of Array.from(files)) {
    byName.set(file.name, file);
  }
  const sceneFile = byName.get(sceneFileName);
  if (sceneFile === undefined) {
    throw new Error('the chosen folder has no scene.json');
  }
  const sceneText = await sceneFile.text();
  const scene = JSON.parse(sceneText) as Record<string, unknown>;
  const frameTable = scene['frame_table'] as Record<string, unknown>;
  const statusTable = scene['scalar_status_table'] as Record<string, unknown>;
  const framesFile = byName.get(String(frameTable['file']));
  const statusesFile = byName.get(String(statusTable['file']));
  if (framesFile === undefined || statusesFile === undefined) {
    throw new Error('the chosen folder lacks the frame or status table named in scene.json');
  }
  const visualFile = scene['visual_definition_file'];
  const visualText =
    typeof visualFile === 'string' ? await (byName.get(visualFile)?.text() ?? Promise.resolve(null)) : null;
  if (typeof visualFile === 'string' && visualText === null) {
    throw new Error(`the chosen folder lacks ${visualFile}`);
  }
  return parseSceneRecord(sceneText, await framesFile.arrayBuffer(), await statusesFile.arrayBuffer(), visualText);
}
