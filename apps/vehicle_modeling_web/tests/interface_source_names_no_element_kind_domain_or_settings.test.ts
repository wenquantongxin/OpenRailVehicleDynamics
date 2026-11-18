import assert from 'node:assert/strict';
import { readdirSync, readFileSync, statSync } from 'node:fs';
import { join, relative } from 'node:path';
import { test } from 'node:test';

import ts from 'typescript';

// The interface draws what the service and its server describe and knows none
// of it: no element kind, no settings word, none of the properties of a rigid
// body and the joints that attach it nor of the three orientations a project
// holds, no domain or row of the model tree, no title the service gives, and
// nothing of the starting points it is offered, their names, titles,
// descriptions and categories, may stand in its source.
// Kind names, property names, starting points' names and the longer settings
// words are distinctive and may stand nowhere, comments included; the two
// short settings words and every title, description and category are looked
// for as a whole text in code and markup, where knowledge of them would act.
// The starting points are read from this app's catalogue, so that every one it
// offers is looked for.

const elementKinds = [
  'rigid_body',
  'fixed_frame',
  'revolute_joint',
  'ball_rpy_joint',
  'weld_joint',
  'translational_spring_damper',
  'roll_spring_damper_couple',
  'series_spring_viscous_damper',
  'point_to_point_series_spring_bilinear_damper',
  'saturated_piecewise_linear_damper',
  'reference_frame_linear_bushing',
  'point_to_point_linear_spring_damper',
  'point_to_point_hysteretic_spring',
  'contact_carrier',
  'contact_interface',
];

const distinctiveSettingsWords = ['gravity', 'vehicle_identity', 'contact_computation'];
const settingsWords = [...distinctiveSettingsWords, 'wheel', 'rail'];

/**
 * The properties of a rigid body and of the joints that attach it: which end is which and what only a free body has
 * come from the service; and the three orientations a project holds, which the service names as rotations.
 */
const propertyNames = [
  'moves_freely_in_world',
  'mass_kilograms',
  'center_of_mass_in_body_frame_meters',
  'inertia_moments_about_center_of_mass_kilogram_square_meters',
  'inertia_products_about_center_of_mass_kilogram_square_meters',
  'mechanical_track_station_layout.station_offset_meters',
  'placement.lateral_offset_in_local_track_frame_meters',
  'placement.vertical_offset_in_local_track_frame_meters',
  'parent_frame_name',
  'child_frame_name',
  'axis_in_parent_frame',
  'damping_newton_metre_seconds_per_radian',
  'default_roll_pitch_yaw_angles_radians',
  'placement.rotation_local_track_from_body',
  'rotation_in_body_frame',
  'rotation_body_from_nonspinning_wheel_profile',
];

/** The titles the service gives, in both languages, as the contracts state them: rows of the model tree, kinds, the attachment's, and property sections. */
const titles = [
  ['Globals', '全局'],
  ['Gravity', '重力加速度'],
  ['Model identity', '模型标识'],
  ['Vehicle layout zero', '车辆布局零点'],
  ['Initial conditions', '初始条件'],
  ['Views', '观察视角'],
  ['Bodies', '刚体'],
  ['Markers', '标记点'],
  ['Geometry', '几何'],
  ['Owner not found', '归属未解析'],
  ['Joints', '铰'],
  ['Force elements', '力元'],
  ['Constraints', '约束'],
  ['Wheel-rail', '轮轨'],
  ['Wheel', '车轮'],
  ['Rail', '钢轨'],
  ['Contact computation', '接触计算'],
  ['Contact carriers', '接触承载体'],
  ['Track', '线路'],
  ['Substructures', '子结构'],
  ['Rigid body', '刚体'],
  ['Marker', '标记点'],
  ['Revolute joint', '转动铰'],
  ['Ball joint', '球铰'],
  ['Weld joint', '固结铰'],
  ['Translational spring-damper', '平动弹簧阻尼'],
  ['Roll spring-damper couple', '侧滚弹簧阻尼力偶'],
  ['Series spring and viscous damper', '串联弹簧粘滞阻尼'],
  ['Point-to-point series spring and bilinear damper', '点对点串联弹簧双线性阻尼'],
  ['Saturated piecewise-linear damper', '饱和分段线性阻尼'],
  ['Linear bushing in the reference frame', '参考坐标系线性衬套'],
  ['Point-to-point linear spring-damper', '点对点线性弹簧阻尼'],
  ['Point-to-point hysteretic spring', '点对点迟滞弹簧'],
  ['Contact carrier', '接触承载体'],
  ['Wheel-rail contact', '轮轨接触'],
  ['Simulation settings', '仿真设置'],
  ['Attachment', '挂接'],
  ['Free in the world', '相对世界自由'],
  ['Moves freely in the world', '相对世界自由'],
  ['Not attached', '未接入'],
  ['Attachment unclear', '挂接不明'],
  ['Joint', '铰'],
  ['Held at', '挂接在'],
  ['Hangs on a joint', '由铰挂接'],
  ['Mass and inertia', '质量与惯量'],
  ['Station along the track, from the vehicle layout zero', '沿线路的站位（相对车辆布局零点）'],
  ['Preview placement', '预览摆放'],
].flat();

/** The starting points of this app's catalogue: their names, and their titles, descriptions and categories in both languages. */
const catalogue = JSON.parse(readFileSync(join(import.meta.dirname, '..', 'project_starting_point_catalogue.json'), 'utf8')) as {
  name: string;
  title: { en: string; zh: string };
  description: { en: string; zh: string };
  category: { en: string; zh: string };
}[];
const startingPointNames = catalogue.map((entry) => entry.name);
const startingPointTexts = catalogue.flatMap((entry) => [entry.title, entry.description, entry.category]).flatMap((text) => [text.en, text.zh]);

const source = join(import.meta.dirname, '..', 'src');

function files(directory: string): string[] {
  return readdirSync(directory).flatMap((name) => {
    const path = join(directory, name);
    return statSync(path).isDirectory() ? files(path) : [path];
  });
}

/** Every string, template text and markup text of a file, trimmed, with its line. */
function textsOf(path: string, content: string): { text: string; line: number }[] {
  const file = ts.createSourceFile(path, content, ts.ScriptTarget.ES2022, true, path.endsWith('.tsx') ? ts.ScriptKind.TSX : ts.ScriptKind.TS);
  const found: { text: string; line: number }[] = [];
  const visit = (node: ts.Node): void => {
    if (ts.isStringLiteralLike(node) || ts.isJsxText(node) || ts.isTemplateLiteralToken(node)) {
      found.push({ text: node.text.trim(), line: file.getLineAndCharacterOfPosition(node.getStart(file)).line + 1 });
    }
    ts.forEachChild(node, visit);
  };
  visit(file);
  return found;
}

test('the interface’s source names no element kind, no settings word, no such property, no row of the model tree, no title the service gives and no starting point', () => {
  const token = (word: string) => new RegExp(`(?<![A-Za-z0-9_.])${word.replaceAll('.', '\\.')}(?![A-Za-z0-9_])`);
  const found: string[] = [];
  for (const path of files(source)) {
    const content = readFileSync(path, 'utf8');
    const where = relative(source, path);
    content.split('\n').forEach((line, index) => {
      for (const word of [...elementKinds, ...distinctiveSettingsWords, ...propertyNames, ...startingPointNames]) if (token(word).test(line)) found.push(`${where}:${index + 1}: ${word}`);
    });
    if (!/\.tsx?$/.test(path)) continue;
    for (const { text, line } of textsOf(path, content)) {
      if (settingsWords.includes(text) || titles.includes(text) || startingPointTexts.includes(text)) found.push(`${where}:${line}: '${text}'`);
    }
  }
  assert.deepEqual(found, []);
});
