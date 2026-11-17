import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
import { test } from 'node:test';

import { parseProjectStartingPointCatalogue, readProjectStartingPointCatalogue } from '../server/read_project_starting_point_catalogue.ts';

// People write the catalogue, so it is read strictly; each entry is handed on
// with the service command that opens its kind of source, paths resolved
// against the catalogue's folder.

const texts = {
  title: { en: 'Made-up A', zh: '虚构甲' },
  description: { en: 'A made-up one.', zh: '虚构的。' },
  category: { en: 'Made-up', zh: '虚构' },
};

const fromFiles = {
  name: 'A',
  ...texts,
  source: {
    vehicle_files: {
      vehicle_definition_path: '../library/a/vehicle_definition.json',
      startup_state_path: '../library/a/startup.json',
      contact_record_path: '../library/a/contact_record.json',
      visual_definition_path: '../library/a/visual_definition.json',
    },
  },
};

const fromTemplate = { name: 'T', ...texts, source: { vehicle_project_template: { directory: '../templates', name: 'made_up' } } };

function parse(entries: unknown): ReturnType<typeof parseProjectStartingPointCatalogue> {
  return parseProjectStartingPointCatalogue(JSON.stringify(entries), '/catalogues/apps');
}

test('entries are read in order with their texts; each opens by its kind of source’s command, its paths relative to the catalogue’s folder', () => {
  const noVisual = { ...fromFiles, name: 'B', source: { vehicle_files: { ...fromFiles.source.vehicle_files, visual_definition_path: null } } };
  const [a, t, b] = parse([fromFiles, fromTemplate, noVisual]);
  assert.deepEqual(a, {
    name: 'A',
    ...texts,
    open: {
      command: 'open_vehicle',
      arguments: {
        vehicle_definition_path: '/catalogues/library/a/vehicle_definition.json',
        startup_state_path: '/catalogues/library/a/startup.json',
        contact_record_path: '/catalogues/library/a/contact_record.json',
        visual_definition_path: '/catalogues/library/a/visual_definition.json',
      },
    },
  });
  assert.deepEqual(t?.open, { command: 'new_project_from_template', arguments: { directory: '/catalogues/templates', name: 'made_up' } }, 'a template’s name is no path');
  assert.equal(b?.open.arguments['visual_definition_path'], null, 'files that name no visual definition open with none');
});

test('a catalogue that cannot be read whole is refused, naming the entry and the key', () => {
  const { visual_definition_path: _, ...filesWithoutVisual } = fromFiles.source.vehicle_files;
  const { description: __, ...withoutDescription } = fromFiles;
  for (const [entries, message] of [
    [{ ...fromFiles }, /does not hold a list/],
    [[{ ...fromFiles, colour: 'red' }], /entry 0 has the unknown key 'colour'/],
    [[withoutDescription], /entry 0 lacks 'description'/],
    [[{ ...fromFiles, name: ' ' }], /entry 0\.name is not a non-empty string/],
    [[{ ...fromFiles, category: { en: 'Made-up' } }], /A\.category lacks 'zh'/],
    [[{ ...fromFiles, title: { en: 'A', zh: '甲', de: 'A' } }], /A\.title has the unknown key 'de'/],
    [[{ ...fromFiles, source: {} }], /A\.source is not one of/],
    [[{ ...fromFiles, source: { ...fromFiles.source, ...fromTemplate.source } }], /A\.source is not one of/],
    [[{ ...fromFiles, source: { blank_project: {} } }], /A\.source has the unknown kind of source 'blank_project'/],
    [[{ ...fromFiles, source: { vehicle_files: filesWithoutVisual } }], /A\.source\.vehicle_files lacks 'visual_definition_path'/],
    [[{ ...fromFiles, source: { vehicle_files: { ...fromFiles.source.vehicle_files, contact_record_path: 3 } } }], /A\.source\.vehicle_files\.contact_record_path/],
    [[{ ...fromTemplate, source: { vehicle_project_template: { directory: '../templates' } } }], /T\.source\.vehicle_project_template lacks 'name'/],
    [[{ ...fromTemplate, source: { vehicle_project_template: { directory: '', name: 'made_up' } } }], /T\.source\.vehicle_project_template\.directory/],
    [[fromFiles, { ...fromTemplate, name: 'A' }], /'A' is listed twice/],
  ] as const) {
    assert.throws(() => parse(entries), message);
  }
  assert.throws(() => parseProjectStartingPointCatalogue('[{"name": "A",', '/catalogues/apps'), /project starting point catalogue: the file is not JSON/);
});

test('this app’s catalogue offers a template and the two reference vehicles, in that order and under two categories', async () => {
  const path = fileURLToPath(new URL('../project_starting_point_catalogue.json', import.meta.url));
  const catalogue = await readProjectStartingPointCatalogue(path);
  assert.deepEqual(
    catalogue.map((entry) => [entry.name, entry.category.en, entry.open.command]),
    [
      ['single_body', 'General', 'new_project_from_template'],
      ['GZ18', 'Rail vehicle', 'open_vehicle'],
      ['IRW', 'Rail vehicle', 'open_vehicle'],
    ],
  );
  assert.ok(String(catalogue[0]!.open.arguments['directory']).endsWith('/tools/vehicle_modeling_service/project_templates'));
});
