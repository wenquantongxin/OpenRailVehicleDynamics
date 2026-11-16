import * as THREE from 'three';

import { elementKey, sameElement, type ElementReference, type Preview, type Rotation, type Vector3 } from '../service/vehicle_modeling_service_protocol.ts';
import type { VisualDefinition, VisualPart, WheelSectionSizes } from '../visual/read_vehicle_visual_definition.ts';
import { buildWheelSection, standardWheelSectionSizes, type WheelSection } from './schematic_wheel_section_geometry.ts';
import { viewColors, type ModelPalette } from './vehicle_model_view_colors_and_materials.ts';
import { axesLines, cylinderBetween, roundedBoxOutline, solidPartMesh } from './visual_definition_part_meshes.ts';

// The three.js drawing of one preview. Everything is built in the drawing
// frame (+x forward, +y right, +z down, metres) under a root group turned the
// way the scene viewer turns its root: +90 degrees about x maps (x, y, z) of
// the drawing frame to (x, -z, y) of three's y-up world, so nothing else
// permutes axes. A rigid body is a group at its pose holding its visual
// definition parts in its own frame. A wheel is a part of the body it turns
// with, standing at its datum: a rigid wheelset's axle and both its wheels are
// one drawing, each independently rotating wheel its own; selecting a wheel's
// contact interface highlights that wheel alone. A body with neither parts
// nor wheels gets a small neutral marker. Joints and force elements are marks
// at their drawn places; fixed frames are small triads, shown on request.
// Every drawing shows the element the preview names for it.
//
// Marks keep a schematic size in metres but are never drawn smaller than a
// few pixels, so that they can be seen and picked in a whole-vehicle view; a
// selected mark is drawn larger and over everything.

export type PartsDisplay = 'solid' | 'xray' | 'hidden';

export interface SceneDisplay {
  parts: PartsDisplay;
  frames: boolean;
}

/** Schematic sizes of the marks, in metres. */
const markSizes = {
  bodyMarkerRadius: 0.07,
  forceBarRadius: 0.014,
  forcePointRadius: 0.045,
  revoluteHalfLength: 0.2,
  revoluteRadius: 0.018,
  ballRadius: 0.05,
  weldHalfEdge: 0.04,
  frameAxisLength: 0.25,
  frameDotRadius: 0.025,
};

/** The least radius a mark is drawn with, in pixels. */
const markPixels = { usual: 3.5, selected: 7 };

/** A click counts on a mark within this multiple of its drawn radius. */
const pickMargin = 1.6;

/** Which display switch a drawing follows: the parts switch, none, or the frames switch. */
type DrawingLayer = 'parts' | 'marks' | 'frames';

interface PartMesh {
  mesh: THREE.Mesh;
  /** A see-through role (shell, glass, floor): drawn as it is in x-ray mode too. */
  translucent: boolean;
  usual: THREE.Material;
  /** The element the part also stands for on its own: a wheel's contact interface. */
  ownElement: ElementReference | null;
}

/** A small shape standing for a body, joint, force element or frame; a bar grows only in thickness. */
interface Mark {
  mesh: THREE.Mesh;
  /** An invisible, slightly larger copy of the shape that clicks have to hit. */
  proxy: THREE.Mesh;
  shape: 'point' | 'bar';
  radiusMeters: number;
  usual: THREE.Material;
}

export interface Drawing {
  element: ElementReference;
  layer: DrawingLayer;
  parts: PartMesh[];
  marks: Mark[];
  /** Further objects shown and hidden with the drawing: a frame's axis lines, a shell's outline. */
  companions: THREE.Object3D[];
}

/** A shape drawn now, which a ray can therefore pick, and how it meets the ray. */
interface PickTarget {
  drawing: Drawing;
  /** A selected mark: drawn over everything, so it comes before whatever is in front of it. */
  overEverything: boolean;
  /** A part drawn see-through while the parts are not shown solid: the ray goes on to what is drawn behind it. */
  letsRayThrough: boolean;
}

function posedGroup(rotation: Rotation, origin: Vector3): THREE.Group {
  const [[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]] = rotation;
  const group = new THREE.Group();
  new THREE.Matrix4()
    .set(r00, r01, r02, origin[0], r10, r11, r12, origin[1], r20, r21, r22, origin[2], 0, 0, 0, 1)
    .decompose(group.position, group.quaternion, group.scale);
  return group;
}

function mark(mesh: THREE.Mesh, shape: Mark['shape'], radiusMeters: number, usual: THREE.Material): Mark {
  const pickRadius = pickMargin * radiusMeters;
  const geometry =
    shape === 'point'
      ? new THREE.SphereGeometry(pickRadius, 10, 8)
      : new THREE.CylinderGeometry(pickRadius, pickRadius, (mesh.geometry as THREE.CylinderGeometry).parameters.height, 10);
  const proxy = new THREE.Mesh(geometry);
  proxy.visible = false;
  proxy.position.copy(mesh.position);
  proxy.quaternion.copy(mesh.quaternion);
  return { mesh, proxy, shape, radiusMeters, usual };
}

const markWorldPosition = new THREE.Vector3();

/** Whether a preview draws anything: one that draws nothing has no extent to frame or to lay a grid under. */
export function drawsAnything(preview: Preview): boolean {
  return [preview.rigidBodies, preview.fixedFrames, preview.revoluteJoints, preview.ballRpyJoints, preview.weldJoints, preview.forceElements, preview.wheels].some(
    (drawings) => drawings.length > 0,
  );
}

/**
 * The visual definition's parts by the drawn rigid body they hang on, keyed by
 * the body's element key, and the parts no drawn body takes: their body is not
 * drawable, or no longer exists. A body takes the parts stated under the name
 * the preview gives for its shapes, which a rename leaves as it was; a body
 * without that name takes none.
 */
export function assignPartsToDrawnBodies(visualDefinition: VisualDefinition | null, preview: Preview): { byBody: Map<string, VisualPart[]>; withoutBody: VisualPart[] } {
  const byBody = new Map<string, VisualPart[]>(preview.rigidBodies.map((body) => [elementKey(body.element), []]));
  const byShapeName = new Map<string, VisualPart[]>();
  for (const body of preview.rigidBodies) {
    if (body.visualDefinitionBodyName !== null) byShapeName.set(body.visualDefinitionBodyName, byBody.get(elementKey(body.element))!);
  }
  const withoutBody: VisualPart[] = [];
  for (const part of visualDefinition?.parts ?? []) {
    const parts = byShapeName.get(part.bodyName);
    if (parts === undefined) withoutBody.push(part);
    else parts.push(part);
  }
  return { byBody, withoutBody };
}

export class VehiclePreviewScene {
  /** Holds everything, turned from the drawing frame into three's world. */
  readonly root = new THREE.Group();
  readonly drawings: Drawing[] = [];
  /** Exactly the shapes drawn now, by the object a ray hits for each: a part's mesh, a mark's proxy. */
  private readonly pickTargets = new Map<THREE.Object3D, PickTarget>();
  private readonly bodyDrawings = new Map<string, Drawing>();
  /** The visual definition's own axes parts, shown with the frames. */
  private readonly bodyAxes: THREE.Object3D[] = [];
  private readonly palette: ModelPalette;
  private display: SceneDisplay = { parts: 'solid', frames: false };
  private highlighted: ElementReference | null = null;

  constructor(preview: Preview, visualDefinition: VisualDefinition | null, palette: ModelPalette) {
    this.palette = palette;
    this.root.rotation.x = Math.PI / 2;
    this.drawBodies(preview, assignPartsToDrawnBodies(visualDefinition, preview).byBody, new Set(preview.wheels.map((wheel) => elementKey(wheel.wheelBody))));
    this.drawWheels(preview, visualDefinition?.wheelSection ?? standardWheelSectionSizes);
    this.drawFrames(preview);
    this.drawJoints(preview);
    this.drawForceElements(preview);
    this.root.updateMatrixWorld(true);
    this.refresh();
  }

  /** Adds a drawing's objects under the parent. */
  private add(drawing: Drawing, parent: THREE.Object3D): void {
    for (const part of drawing.parts) parent.add(part.mesh);
    for (const { mesh, proxy } of drawing.marks) parent.add(mesh, proxy);
    for (const companion of drawing.companions) parent.add(companion);
    this.drawings.push(drawing);
  }

  private drawBodies(preview: Preview, partsByBody: Map<string, VisualPart[]>, bodiesWithWheels: ReadonlySet<string>): void {
    for (const body of preview.rigidBodies) {
      const group = posedGroup(body.rotationDrawingFromLocal, body.originInDrawingFrameMeters);
      this.root.add(group);
      const drawing: Drawing = { element: body.element, layer: 'parts', parts: [], marks: [], companions: [] };
      for (const part of partsByBody.get(elementKey(body.element))!) {
        if (part.kind === 'axes') {
          const axes = axesLines(part.lengthMeters, [viewColors.axisX, viewColors.axisY, viewColors.axisZ], this.palette.frameAxes);
          axes.position.set(...part.originInBodyFrameMeters);
          group.add(axes);
          this.bodyAxes.push(axes);
          continue;
        }
        const usual = this.palette.byAppearance[part.appearance];
        const mesh = solidPartMesh(part, usual);
        const translucent = part.appearance === 'shell' || part.appearance === 'glass' || part.appearance === 'floor';
        drawing.parts.push({ mesh, translucent, usual, ownElement: null });
        if (part.kind === 'box' && part.appearance === 'shell') {
          const outline = new THREE.LineSegments(roundedBoxOutline(part.sizeMeters, part.cornerRadiusMeters), this.palette.shellOutline);
          outline.position.set(...part.centerInBodyFrameMeters);
          outline.renderOrder = 4;
          drawing.companions.push(outline);
        }
      }
      if (drawing.parts.length === 0 && !bodiesWithWheels.has(elementKey(body.element))) {
        const marker = new THREE.Mesh(new THREE.SphereGeometry(markSizes.bodyMarkerRadius, 16, 12), this.palette.bodyMarker);
        drawing.marks.push(mark(marker, 'point', markSizes.bodyMarkerRadius, this.palette.bodyMarker));
      }
      this.add(drawing, group);
      this.bodyDrawings.set(elementKey(body.element), drawing);
    }
  }

  /** Each wheel turned about its outward spin axis, the lathe's +y, at its datum in the drawing frame. */
  private drawWheels(preview: Preview, sizes: WheelSectionSizes): void {
    const sections = new Map<number, WheelSection>();
    for (const wheel of preview.wheels) {
      const body = this.bodyDrawings.get(elementKey(wheel.wheelBody));
      if (body === undefined) {
        throw new Error(`the preview draws a wheel of '${wheel.element.name}' on '${wheel.wheelBody.name}', which it does not draw`);
      }
      const radius = wheel.nominalRollingRadiusMeters;
      let section = sections.get(radius);
      if (section === undefined) {
        section = buildWheelSection(radius, sizes);
        sections.set(radius, section);
      }
      const group = new THREE.Group();
      group.position.set(...wheel.datumInDrawingFrameMeters);
      group.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(...wheel.outwardSpinAxisInDrawingFrame));
      this.root.add(group);
      for (const [geometry, usual] of [
        [section.body, this.palette.wheelBody],
        [section.tread, this.palette.wheelTread],
      ] as const) {
        const mesh = new THREE.Mesh(geometry, usual);
        group.add(mesh);
        body.parts.push({ mesh, translucent: false, usual, ownElement: wheel.element });
      }
    }
  }

  private drawFrames(preview: Preview): void {
    for (const frame of preview.fixedFrames) {
      const group = posedGroup(frame.rotationDrawingFromLocal, frame.originInDrawingFrameMeters);
      this.root.add(group);
      const dot = new THREE.Mesh(new THREE.SphereGeometry(markSizes.frameDotRadius, 12, 8), this.palette.frameDot);
      this.add(
        {
          element: frame.element,
          layer: 'frames',
          parts: [],
          marks: [mark(dot, 'point', markSizes.frameDotRadius, this.palette.frameDot)],
          companions: [axesLines(markSizes.frameAxisLength, [viewColors.axisX, viewColors.axisY, viewColors.axisZ], this.palette.frameAxes)],
        },
        group,
      );
    }
  }

  private addMark(element: ElementReference, drawn: Mark): void {
    this.add({ element, layer: 'marks', parts: [], marks: [drawn], companions: [] }, this.root);
  }

  private drawJoints(preview: Preview): void {
    const joint = this.palette.joint;
    for (const revolute of preview.revoluteJoints) {
      const location = new THREE.Vector3(...revolute.locationInDrawingFrameMeters);
      const half = new THREE.Vector3(...revolute.axisInDrawingFrame).multiplyScalar(markSizes.revoluteHalfLength);
      const axis = cylinderBetween(location.clone().sub(half), location.clone().add(half), markSizes.revoluteRadius, joint);
      this.addMark(revolute.element, mark(axis, 'bar', markSizes.revoluteRadius, joint));
    }
    for (const ball of preview.ballRpyJoints) {
      const sphere = new THREE.Mesh(new THREE.SphereGeometry(markSizes.ballRadius, 16, 12), joint);
      sphere.position.set(...ball.locationInDrawingFrameMeters);
      this.addMark(ball.element, mark(sphere, 'point', markSizes.ballRadius, joint));
    }
    for (const weld of preview.weldJoints) {
      const edge = 2 * markSizes.weldHalfEdge;
      const cube = new THREE.Mesh(new THREE.BoxGeometry(edge, edge, edge), joint);
      cube.position.set(...weld.locationInDrawingFrameMeters);
      this.addMark(weld.element, mark(cube, 'point', markSizes.weldHalfEdge, joint));
    }
  }

  private drawForceElements(preview: Preview): void {
    const force = this.palette.forceElement;
    for (const drawing of preview.forceElements) {
      const reference = new THREE.Vector3(...drawing.referenceEndInDrawingFrameMeters);
      const opposite = new THREE.Vector3(...drawing.oppositeEndInDrawingFrameMeters);
      // Ends closer than a point mark is wide would draw a bar hidden inside it: draw the point.
      if (reference.distanceTo(opposite) < 2 * markSizes.forcePointRadius) {
        const point = new THREE.Mesh(new THREE.OctahedronGeometry(markSizes.forcePointRadius), force);
        point.position.copy(reference).add(opposite).multiplyScalar(0.5);
        this.addMark(drawing.element, mark(point, 'point', markSizes.forcePointRadius, force));
      } else {
        const bar = cylinderBetween(reference, opposite, markSizes.forceBarRadius, force);
        this.addMark(drawing.element, mark(bar, 'bar', markSizes.forceBarRadius, force));
      }
    }
  }

  setDisplay(display: SceneDisplay): void {
    this.display = display;
    this.refresh();
  }

  setHighlight(element: ElementReference | null): void {
    this.highlighted = element;
    this.refresh();
  }

  /**
   * Whether the display switches show the drawing now; a selected drawing they always show. A part standing for a
   * selected element of its own, a wheel for its contact interface, is drawn even when its drawing is not shown.
   */
  shown(drawing: Drawing): boolean {
    if (sameElement(drawing.element, this.highlighted)) return true;
    if (drawing.layer === 'parts') return this.display.parts !== 'hidden';
    if (drawing.layer === 'frames') return this.display.frames;
    return true;
  }

  /**
   * The drawing a ray points at. Only what is drawn now can be picked. The ray meets the nearest drawn shape, and a
   * selected mark first, since it is drawn over everything. While the parts are shown as x-ray or hidden, a part
   * drawn see-through lets the ray on to what is drawn behind it, and is picked only where nothing is.
   */
  pick(raycaster: THREE.Raycaster): Drawing | null {
    const hits = raycaster.intersectObjects([...this.pickTargets.keys()], false).map((hit) => this.pickTargets.get(hit.object)!);
    const picked = hits.find((hit) => hit.overEverything) ?? hits.find((hit) => !hit.letsRayThrough) ?? hits[0];
    return picked?.drawing ?? null;
  }

  /** Grows every mark to its least size on screen for this camera; call before rendering. */
  fitMarksToScreen(camera: THREE.PerspectiveCamera, heightPixels: number): void {
    const metersPerPixelPerMeter = (2 * Math.tan(THREE.MathUtils.degToRad(camera.fov) / 2)) / heightPixels;
    for (const drawing of this.drawings) {
      const pixels = sameElement(drawing.element, this.highlighted) ? markPixels.selected : markPixels.usual;
      for (const each of drawing.marks) {
        const metersPerPixel = each.mesh.getWorldPosition(markWorldPosition).distanceTo(camera.position) * metersPerPixelPerMeter;
        const factor = Math.max(1, (pixels * metersPerPixel) / each.radiusMeters);
        for (const object of [each.mesh, each.proxy]) {
          if (each.shape === 'point') object.scale.setScalar(factor);
          else object.scale.set(factor, 1, factor);
        }
      }
    }
  }

  /** Draws every shape as the display and the highlight say, and makes exactly the drawn shapes pickable. */
  private refresh(): void {
    const palette = this.palette;
    const xray = this.display.parts === 'xray';
    const partsSolid = this.display.parts === 'solid';
    this.pickTargets.clear();
    for (const drawing of this.drawings) {
      const visible = this.shown(drawing);
      const selected = sameElement(drawing.element, this.highlighted);
      for (const part of drawing.parts) {
        const partSelected = selected || sameElement(part.ownElement, this.highlighted);
        const seeThrough = part.translucent || xray;
        part.mesh.visible = visible || partSelected;
        if (partSelected) part.mesh.material = seeThrough ? palette.selectedTranslucentPart : palette.selectedPart;
        else part.mesh.material = xray && !part.translucent ? palette.ghost : part.usual;
        if (part.mesh.visible) this.pickTargets.set(part.mesh, { drawing, overEverything: false, letsRayThrough: seeThrough && !partsSolid });
      }
      for (const each of drawing.marks) {
        each.mesh.visible = visible;
        each.mesh.material = selected ? palette.selectedMark : each.usual;
        // A selected mark is drawn last, over everything.
        each.mesh.renderOrder = selected ? 10 : 0;
        if (visible) this.pickTargets.set(each.proxy, { drawing, overEverything: selected, letsRayThrough: false });
      }
      for (const companion of drawing.companions) companion.visible = visible;
    }
    for (const axes of this.bodyAxes) axes.visible = this.display.frames;
  }

  /** The bounding box of everything drawn, in three's world. */
  bounds(): THREE.Box3 {
    return new THREE.Box3().setFromObject(this.root);
  }

  dispose(): void {
    this.root.traverse((object) => {
      if (object instanceof THREE.Mesh || object instanceof THREE.LineSegments) {
        (object.geometry as THREE.BufferGeometry).dispose();
      }
    });
  }
}
