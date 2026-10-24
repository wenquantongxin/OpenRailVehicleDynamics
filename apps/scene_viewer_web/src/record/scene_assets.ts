import * as field from './json_fields.ts';
import type { RailDatum, SceneDefinition, SceneDescription, TrackTable } from './scene_definition.ts';
import { parseVisualDefinition } from './visual_definition.ts';
import { resolveVehicleDisplayBindings } from '../scene/vehicle_display_bindings.ts';

/** Resource metadata plus already-read content; never owns topology or samples. */
export interface SceneAssets {
  worldFrame: string;
  lengthUnit: 'meter';
  angleUnit: 'radian';
  track: TrackTable | null;
  visualDefinitionFile: 'visual_definition.json' | null;
  visualDefinitionText: string | null;
}

function numberRows(value: unknown, width: number, what: string): number[][] {
  return field.array(value, what).map((entry) => {
    const row = field.array(entry, what);
    if (row.length !== width) throw new Error(`${what} rows must have ${width} entries`);
    return row.map((number) => field.number(number, what));
  });
}
function numbers(value: unknown, what: string): number[] {
  return field.array(value, what).map((entry) => field.number(entry, what));
}
function railDatum(value: unknown, what: string): RailDatum {
  const datum = field.object(value, what);
  return { lateralMeters: field.number(datum['lateral_meters'], `${what}.lateral_meters`),
    verticalMeters: field.number(datum['vertical_meters'], `${what}.vertical_meters`) };
}

export function parseSceneAssets(value: unknown, visualDefinitionText: string | null = null): SceneAssets {
  const assets = field.object(value, 'assets');
  field.exact(assets['length_unit'], 'meter', 'assets.length_unit');
  field.exact(assets['angle_unit'], 'radian', 'assets.angle_unit');
  const visualFile = assets['visual_definition_file'];
  if (visualFile !== null && visualFile !== 'visual_definition.json') throw new Error('assets.visual_definition_file must be null or visual_definition.json');
  let track: TrackTable | null = null;
  if (assets['track'] !== null) {
    const table = field.object(assets['track'], 'assets.track');
    const datums = field.object(table['rail_datums'], 'track.rail_datums');
    track = {
      stationsMeters: numbers(table['stations_meters'], 'track.stations_meters'),
      centerlineInInertialMeters: numberRows(table['centerline_in_inertial_meters'], 3, 'track.centerline'),
      rotationInertialFromTrackWxyz: numberRows(table['rotation_inertial_from_track_wxyz'], 4, 'track.rotation'),
      curvatureRadiansPerMeter: numbers(table['curvature_radians_per_meter'], 'track.curvature'),
      superelevationMeters: numbers(table['superelevation_meters'], 'track.superelevation'),
      leftRailDatumInInertialMeters: numberRows(table['left_rail_datum_in_inertial_meters'], 3, 'track.left rail'),
      rightRailDatumInInertialMeters: numberRows(table['right_rail_datum_in_inertial_meters'], 3, 'track.right rail'),
      railDatums: { left: railDatum(datums['left'], 'track.rail_datums.left'), right: railDatum(datums['right'], 'track.rail_datums.right') },
    };
    const count = track.stationsMeters.length;
    if (count === 0 || [track.centerlineInInertialMeters, track.rotationInertialFromTrackWxyz,
      track.curvatureRadiansPerMeter, track.superelevationMeters, track.leftRailDatumInInertialMeters,
      track.rightRailDatumInInertialMeters].some((column) => column.length !== count)) {
      throw new Error('track table columns disagree in length');
    }
    if (track.stationsMeters.some((station, index, entries) => index > 0 && station <= entries[index - 1]!)) throw new Error('track stations must be strictly increasing');
    if (track.rotationInertialFromTrackWxyz.some((row) => Math.abs(row.reduce((sum, value) => sum + value * value, 0) - 1) > 1e-8)) throw new Error('track rotations must be unit quaternions');
  }
  return { worldFrame: field.string(assets['world_frame'], 'assets.world_frame'), lengthUnit: 'meter', angleUnit: 'radian',
    track, visualDefinitionFile: visualFile, visualDefinitionText };
}

/** Pure composition: a bundle replaces only resources that it actually provides. */
export function composeSceneDefinition(description: SceneDescription, recordAssets: SceneAssets, bundleAssets: SceneAssets | null = null): SceneDefinition {
  for (const assets of [recordAssets, bundleAssets]) {
    if (assets === null) continue;
    if (assets.worldFrame !== description.worldFrame || assets.lengthUnit !== description.lengthUnit || assets.angleUnit !== description.angleUnit) {
      throw new Error('scene assets coordinate conventions disagree with the description');
    }
  }
  const visualAssets = bundleAssets !== null && bundleAssets.visualDefinitionFile !== null ? bundleAssets : recordAssets;
  if (visualAssets.visualDefinitionFile !== null && visualAssets.visualDefinitionText === null) throw new Error('scene assets lack the named visual_definition.json');
  const definition: SceneDefinition = { ...description,
    track: bundleAssets?.track ?? recordAssets.track,
    visualDefinitionText: visualAssets.visualDefinitionFile === null ? null : visualAssets.visualDefinitionText };
  const visual = definition.visualDefinitionText === null ? null : parseVisualDefinition(definition.visualDefinitionText, new Set(description.bodies.map((body) => body.name)));
  resolveVehicleDisplayBindings(definition, visual);
  const interfaceNames = new Set(description.wheelPlacements.map((wheel) => wheel.interfaceName));
  if (visual?.wheelVisual.labels.some((label) => !interfaceNames.has(label.interfaceName))) throw new Error('visual wheel label names an unknown wheel interface');
  return definition;
}
