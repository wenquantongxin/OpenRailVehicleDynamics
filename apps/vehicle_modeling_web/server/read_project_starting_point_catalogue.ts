import { readFile } from 'node:fs/promises';
import { dirname, resolve } from 'node:path';

// The starting points a new project can start from, from a catalogue file
// people write: a JSON list with one entry per starting point,
//
//   {"name": "single_body",
//    "title": {"en": "...", "zh": "..."},
//    "description": {"en": "...", "zh": "..."},
//    "category": {"en": "...", "zh": "..."},
//    "source": {"vehicle_project_template": {"directory": "...", "name": "..."}}
//           or {"vehicle_files": {"vehicle_definition_path": "...", "startup_state_path": "...",
//                                 "contact_record_path": "...", "visual_definition_path": "..." or null}}}
//
// with every path relative to the catalogue's own folder. A template is a
// project file kept with the modelling service, which opens a new session over
// it; a vehicle's own files are opened as they are. Each entry is handed on
// with the service command that opens it and the command's arguments, which
// the interface sends as given. Because people write the catalogue, it is read
// strictly: exactly these keys, non-empty strings, one kind of source, and no
// name listed twice; a catalogue that cannot be read whole is refused, naming
// the entry and the key.

export interface BilingualCatalogueText {
  en: string;
  zh: string;
}

/** A starting point as the interface is given it: what it is called, and the service command that opens it, with absolute paths. */
export interface ProjectStartingPointCatalogueEntry {
  name: string;
  title: BilingualCatalogueText;
  description: BilingualCatalogueText;
  category: BilingualCatalogueText;
  open: { command: string; arguments: Readonly<Record<string, string | null>> };
}

function refused(what: string, problem: string): Error {
  return new Error(`project starting point catalogue: ${what} ${problem}`);
}

function fieldsOf(value: unknown, keys: readonly string[], what: string): Record<string, unknown> {
  if (typeof value !== 'object' || value === null || Array.isArray(value)) throw refused(what, 'is not an object');
  const fields = value as Record<string, unknown>;
  for (const key of Object.keys(fields)) {
    if (!keys.includes(key)) throw refused(what, `has the unknown key '${key}'`);
  }
  for (const key of keys) {
    if (!(key in fields)) throw refused(what, `lacks '${key}'`);
  }
  return fields;
}

function text(value: unknown, what: string): string {
  if (typeof value !== 'string' || value.trim() === '') throw refused(what, 'is not a non-empty string');
  return value;
}

function bilingual(value: unknown, what: string): BilingualCatalogueText {
  const fields = fieldsOf(value, ['en', 'zh'], what);
  return { en: text(fields['en'], `${what}.en`), zh: text(fields['zh'], `${what}.zh`) };
}

/** The command that opens a source and its arguments, its paths made absolute against the catalogue's folder. */
function openingOf(value: unknown, what: string, catalogueDirectory: string): ProjectStartingPointCatalogueEntry['open'] {
  if (typeof value !== 'object' || value === null || Array.isArray(value)) throw refused(what, 'is not an object');
  const kinds = Object.keys(value);
  if (kinds.length !== 1) throw refused(what, "is not one of 'vehicle_project_template' and 'vehicle_files'");
  const kind = kinds[0]!;
  const source = (value as Record<string, unknown>)[kind];
  const path = (fields: Record<string, unknown>, key: string): string => resolve(catalogueDirectory, text(fields[key], `${what}.${kind}.${key}`));
  if (kind === 'vehicle_project_template') {
    const fields = fieldsOf(source, ['directory', 'name'], `${what}.${kind}`);
    return { command: 'new_project_from_template', arguments: { directory: path(fields, 'directory'), name: text(fields['name'], `${what}.${kind}.name`) } };
  }
  if (kind === 'vehicle_files') {
    const fields = fieldsOf(source, ['vehicle_definition_path', 'startup_state_path', 'contact_record_path', 'visual_definition_path'], `${what}.${kind}`);
    return {
      command: 'open_vehicle',
      arguments: {
        vehicle_definition_path: path(fields, 'vehicle_definition_path'),
        startup_state_path: path(fields, 'startup_state_path'),
        contact_record_path: path(fields, 'contact_record_path'),
        visual_definition_path: fields['visual_definition_path'] === null ? null : path(fields, 'visual_definition_path'),
      },
    };
  }
  throw refused(what, `has the unknown kind of source '${kind}'`);
}

export function parseProjectStartingPointCatalogue(catalogueText: string, catalogueDirectory: string): ProjectStartingPointCatalogueEntry[] {
  let listed: unknown;
  try {
    listed = JSON.parse(catalogueText);
  } catch (error) {
    throw refused('the file', `is not JSON: ${error instanceof Error ? error.message : String(error)}`);
  }
  if (!Array.isArray(listed)) throw refused('the file', 'does not hold a list of starting points');
  const names = new Set<string>();
  return listed.map((entry, index): ProjectStartingPointCatalogueEntry => {
    const fields = fieldsOf(entry, ['name', 'title', 'description', 'category', 'source'], `entry ${index}`);
    const name = text(fields['name'], `entry ${index}.name`);
    if (names.has(name)) throw refused(`'${name}'`, 'is listed twice');
    names.add(name);
    return {
      name,
      title: bilingual(fields['title'], `${name}.title`),
      description: bilingual(fields['description'], `${name}.description`),
      category: bilingual(fields['category'], `${name}.category`),
      open: openingOf(fields['source'], `${name}.source`, catalogueDirectory),
    };
  });
}

export async function readProjectStartingPointCatalogue(path: string): Promise<ProjectStartingPointCatalogueEntry[]> {
  return parseProjectStartingPointCatalogue(await readFile(path, 'utf8'), dirname(path));
}
