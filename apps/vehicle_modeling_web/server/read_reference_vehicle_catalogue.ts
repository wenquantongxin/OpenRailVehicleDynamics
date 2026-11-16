import { readFile } from 'node:fs/promises';
import { dirname, resolve } from 'node:path';

// The reference vehicles the interface offers, from a catalogue file people
// write: a JSON list with one entry per vehicle,
//
//   {"name": "GZ18",
//    "title": {"en": "Bogie vehicle with rigid wheelsets", "zh": "刚性轮对转向架车辆"},
//    "vehicle_definition_path": "...", "startup_state_path": "...",
//    "contact_record_path": "...", "visual_definition_path": "..."}
//
// with visual_definition_path optional and every path relative to the
// catalogue's own folder. Because people write it, it is read strictly:
// exactly these keys, non-empty strings, and no name listed twice.

/** A vehicle the interface can open: its title, the open_vehicle arguments as the service takes them, and its drawing. */
export interface ReferenceVehicleCatalogueEntry {
  name: string;
  title: { en: string; zh: string };
  openArguments: { vehicle_definition_path: string; startup_state_path: string; contact_record_path: string };
  visualDefinitionPath: string | null;
}

function fieldsOf(value: unknown, required: readonly string[], optional: readonly string[], what: string): Record<string, unknown> {
  if (typeof value !== 'object' || value === null || Array.isArray(value)) {
    throw new Error(`reference vehicle catalogue: ${what} is not an object`);
  }
  const fields = value as Record<string, unknown>;
  for (const key of Object.keys(fields)) {
    if (!required.includes(key) && !optional.includes(key)) throw new Error(`reference vehicle catalogue: ${what} has the unknown key '${key}'`);
  }
  for (const key of required) {
    if (!(key in fields)) throw new Error(`reference vehicle catalogue: ${what} lacks '${key}'`);
  }
  return fields;
}

function text(value: unknown, what: string): string {
  if (typeof value !== 'string' || value.trim() === '') {
    throw new Error(`reference vehicle catalogue: ${what} is not a non-empty string`);
  }
  return value;
}

export function parseReferenceVehicleCatalogue(catalogueText: string, catalogueDirectory: string): ReferenceVehicleCatalogueEntry[] {
  const listed: unknown = JSON.parse(catalogueText);
  if (!Array.isArray(listed)) {
    throw new Error('reference vehicle catalogue: the file does not hold a list of vehicles');
  }
  const names = new Set<string>();
  return listed.map((entry, index): ReferenceVehicleCatalogueEntry => {
    const what = `entry ${index}`;
    const fields = fieldsOf(entry, ['name', 'title', 'vehicle_definition_path', 'startup_state_path', 'contact_record_path'], ['visual_definition_path'], what);
    const name = text(fields['name'], `${what}.name`);
    if (names.has(name)) throw new Error(`reference vehicle catalogue: '${name}' is listed twice`);
    names.add(name);
    const title = fieldsOf(fields['title'], ['en', 'zh'], [], `${name}.title`);
    const path = (key: string): string => resolve(catalogueDirectory, text(fields[key], `${name}.${key}`));
    return {
      name,
      title: { en: text(title['en'], `${name}.title.en`), zh: text(title['zh'], `${name}.title.zh`) },
      openArguments: {
        vehicle_definition_path: path('vehicle_definition_path'),
        startup_state_path: path('startup_state_path'),
        contact_record_path: path('contact_record_path'),
      },
      visualDefinitionPath: 'visual_definition_path' in fields ? path('visual_definition_path') : null,
    };
  });
}

export async function readReferenceVehicleCatalogue(path: string): Promise<ReferenceVehicleCatalogueEntry[]> {
  return parseReferenceVehicleCatalogue(await readFile(path, 'utf8'), dirname(path));
}
