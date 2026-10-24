import assert from 'node:assert/strict';
import { test, type TestContext } from 'node:test';
import { loadRecordFromFiles, loadRecordFromUrl } from '../src/record/load_record.ts';
import { loadBundleFromUrl } from '../src/record/load_bundle.ts';
import { contractFixture, fixtureVisual, trackJson } from './scene_contract_fixtures.ts';
import { straightTable } from './track_fixtures.ts';

function serve(t: TestContext, entries: Record<string, string | ArrayBuffer>, signal?: AbortSignal) {
  const requested: string[] = [];
  const previousLocation = Object.getOwnPropertyDescriptor(globalThis, 'location');
  Object.defineProperty(globalThis, 'location', { configurable: true, value: { href: 'https://viewer.test/app/index.html' } });
  t.after(() => {
    if (previousLocation === undefined) Reflect.deleteProperty(globalThis, 'location');
    else Object.defineProperty(globalThis, 'location', previousLocation);
  });
  t.mock.method(globalThis, 'fetch', async (input: string | URL | Request, init?: RequestInit) => {
    const url = String(input);
    requested.push(url);
    if (signal !== undefined) assert.equal(init?.signal, signal);
    if (init?.signal?.aborted) throw new DOMException('aborted', 'AbortError');
    const body = entries[url];
    return body === undefined ? new Response('missing', { status: 404 }) : new Response(body);
  });
  return requested;
}

function recordResources(visual = false) {
  const fixture = contractFixture(visual ? { visualDefinitionText: fixtureVisual('record') } : {});
  const root = 'https://viewer.test/app/records/run/';
  return { ...fixture, entries: { [`${root}scene.json`]: JSON.stringify(fixture.json), [`${root}frames.f64le`]: fixture.frames,
    [`${root}scalar_statuses.u8`]: fixture.statuses, ...(visual ? { [`${root}visual_definition.json`]: fixtureVisual('record') } : {}) } };
}

test('record and bundle paths resolve independently against the page and visual override skips record visual', async (t) => {
  const fixture = recordResources(true);
  const signal = new AbortController().signal;
  const entries = { ...fixture.entries,
    'https://viewer.test/app/resources/scene.json': JSON.stringify({ schema: 'orvd.scene_bundle', assets: { ...fixture.json.assets,
      track: trackJson(straightTable([40, 50], () => 0)), visual_definition_file: 'visual_definition.json' } }),
    'https://viewer.test/app/resources/visual_definition.json': fixtureVisual('bundle'),
  };
  const requested = serve(t, entries, signal);
  const record = await loadRecordFromUrl('records/run/', signal, 'resources/');
  assert.deepEqual(record.definition.track?.stationsMeters, [40, 50]);
  assert.equal(JSON.parse(record.definition.visualDefinitionText!).vehicle_name, 'bundle');
  assert.equal(record.samples.viewAt(1).wheelSpinAngle(0), -12);
  assert.ok(!requested.includes('https://viewer.test/app/records/run/visual_definition.json'));
  assert.equal(requested.length, 5);
});

test('track-only bundle retains and fetches the record visual', async (t) => {
  const fixture = recordResources(true);
  const requested = serve(t, { ...fixture.entries,
    'https://viewer.test/app/resources/scene.json': JSON.stringify({ schema: 'orvd.scene_bundle', assets: { ...fixture.json.assets,
      track: trackJson(straightTable([0, 10], () => 0)), visual_definition_file: null } }),
  });
  const record = await loadRecordFromUrl('records/run', new AbortController().signal, 'resources');
  assert.equal(JSON.parse(record.definition.visualDefinitionText!).vehicle_name, 'record');
  assert.ok(requested.includes('https://viewer.test/app/records/run/visual_definition.json'));
  assert.ok(!requested.includes('https://viewer.test/app/resources/visual_definition.json'));
});

test('visual-only bundle supplies appearance to a record without resources', async (t) => {
  const fixture = recordResources();
  serve(t, { ...fixture.entries,
    'https://viewer.test/app/resources/scene.json': JSON.stringify({ schema: 'orvd.scene_bundle', assets: { ...fixture.json.assets, visual_definition_file: 'visual_definition.json' } }),
    'https://viewer.test/app/resources/visual_definition.json': fixtureVisual('bundle'),
  });
  const record = await loadRecordFromUrl('records/run/', new AbortController().signal, 'resources/');
  assert.equal(record.definition.track, null);
  assert.equal(JSON.parse(record.definition.visualDefinitionText!).vehicle_name, 'bundle');
});

test('bundle loader returns resources alone and propagates fetch and cancellation failures', async (t) => {
  const { json } = contractFixture();
  const requested = serve(t, {
    'https://viewer.test/app/resources/scene.json': JSON.stringify({ schema: 'orvd.scene_bundle', assets: json.assets }),
  });
  const bundle = await loadBundleFromUrl('resources/', new AbortController().signal);
  assert.equal(bundle.track, null);
  assert.equal(bundle.visualDefinitionText, null);
  assert.equal('bodies' in bundle, false);
  assert.equal(requested.length, 1);
  await assert.rejects(loadBundleFromUrl('missing/', new AbortController().signal), /404/);
  const controller = new AbortController();
  controller.abort();
  await assert.rejects(loadBundleFromUrl('resources/', controller.signal), /aborted/);
});

test('a wrong bundle schema or a missing selected visual is an error', async (t) => {
  const { json } = contractFixture();
  serve(t, {
    'https://viewer.test/app/wrong/scene.json': JSON.stringify(json),
    'https://viewer.test/app/resources/scene.json': JSON.stringify({ schema: 'orvd.scene_bundle', assets: { ...json.assets, visual_definition_file: 'visual_definition.json' } }),
  });
  await assert.rejects(loadBundleFromUrl('wrong/', new AbortController().signal), /bundle.schema/);
  await assert.rejects(loadBundleFromUrl('resources/', new AbortController().signal), /visual_definition.json: 404/);
});

test('folder loading stays self-contained and requires any declared visual file', async () => {
  const { json, frames, statuses } = contractFixture({ visualDefinitionText: fixtureVisual() });
  const files = [new File([JSON.stringify(json)], 'scene.json'), new File([frames], 'frames.f64le'), new File([statuses], 'scalar_statuses.u8')];
  await assert.rejects(loadRecordFromFiles(files as unknown as FileList), /lacks visual_definition/);
  files.push(new File([fixtureVisual()], 'visual_definition.json'));
  const record = await loadRecordFromFiles(files as unknown as FileList);
  assert.equal(record.samples.frameCount, 2);
  assert.equal(record.definition.visualDefinitionText, fixtureVisual());
  await assert.rejects(loadRecordFromFiles([new File(['{}'], 'bundle.json')] as unknown as FileList), /choose a record folder/);
});

test('an explicit scene.json URL loads sibling files and record cancellation is propagated', async (t) => {
  const fixture = recordResources();
  const requested = serve(t, { ...fixture.entries,
    'https://viewer.test/app/records/run/scene.json?view=1': JSON.stringify(fixture.json),
  });
  const record = await loadRecordFromUrl('records/run/scene.json?view=1', new AbortController().signal);
  assert.equal(record.samples.frameCount, 2);
  assert.ok(requested.includes('https://viewer.test/app/records/run/frames.f64le'));
  const controller = new AbortController();
  controller.abort();
  await assert.rejects(loadRecordFromUrl('records/run/', controller.signal), /aborted/);
});
