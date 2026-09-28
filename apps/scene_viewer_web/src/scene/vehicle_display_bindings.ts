import type { SceneRecord, WheelSide } from '../record/scene_record.ts';
import type { BilingualName, VisualDefinition } from '../record/visual_definition.ts';

// One resolution of the record's explicit relations and the visual
// definition's display bindings into the indices every consumer shares: the
// readout cards, scene grouping, camera anchors and label obstacles. Nothing
// here infers a role from a body's name or position. Naming a body the record
// does not list is an input error and is refused here; a role the definition
// leaves out is an absence the consumers show as unavailable.

export interface BoundBody {
  bodyName: string;
  bodyIndex: number;
  displayName: BilingualName;
}

export interface BoundCarrier extends BoundBody {
  /** Index into `record.wheelPlacements` of the carrier's wheel of that side; null when it has none. */
  leftWheelPlacementIndex: number | null;
  rightWheelPlacementIndex: number | null;
  /** Index into `bogies` of the display group whose member list names this carrier; null when none does. */
  bogieIndex: number | null;
}

export interface BoundBogie extends BoundBody {
  /** Body indices of this display group: the bogie body, its listed members and the wheels of the listed carriers. */
  memberBodyIndices: number[];
}

export interface VehicleDisplayBindings {
  hasVisualDefinition: boolean;
  carbody: BoundBody | null;
  /** In the visual definition's order, which is the display order from end 1. */
  bogies: BoundBogie[];
  /** In the visual definition's order; without one, in order of first appearance among the wheel placements. */
  carriers: BoundCarrier[];
  /** Display group of every body, by body index; -1 when no group lists it. */
  bogieIndexOfBody: Int32Array;
}

interface CarrierWheels {
  bodyIndex: number;
  left: number | null;
  right: number | null;
}

export function resolveVehicleDisplayBindings(record: SceneRecord, visualDefinition: VisualDefinition | null): VehicleDisplayBindings {
  const bodyIndexByName = new Map(record.bodies.map((body, index) => [body.name, index]));
  const requireBody = (name: string, what: string): number => {
    const index = bodyIndexByName.get(name);
    if (index === undefined) {
      throw new Error(`${what} names body '${name}', which the record does not list`);
    }
    return index;
  };

  // Carriers come from the record's wheel placements, in order of first appearance.
  const carrierWheels = new Map<string, CarrierWheels>();
  record.wheelPlacements.forEach((placement, placementIndex) => {
    const what = `wheel placement '${placement.interfaceName}'`;
    requireBody(placement.wheelBodyName, what);
    const bodyIndex = requireBody(placement.carrierBodyName, what);
    let entry = carrierWheels.get(placement.carrierBodyName);
    if (entry === undefined) {
      entry = { bodyIndex, left: null, right: null };
      carrierWheels.set(placement.carrierBodyName, entry);
    }
    const side: WheelSide = placement.side;
    if (entry[side] !== null) {
      throw new Error(`carrier '${placement.carrierBodyName}' has two ${side} wheels`);
    }
    entry[side] = placementIndex;
  });

  const bindings = visualDefinition?.displayBindings ?? null;
  const carrierOrder: { name: string; displayName: BilingualName }[] =
    bindings === null
      ? [...carrierWheels.keys()].map((name) => ({ name, displayName: { en: name, zh: name } }))
      : bindings.carriers.map((carrier) => ({ name: carrier.carrierBodyName, displayName: carrier.displayName }));
  if (bindings !== null) {
    const listed = new Set<string>();
    for (const { name } of carrierOrder) {
      if (!carrierWheels.has(name)) {
        throw new Error(`display bindings name carrier '${name}', which no wheel placement uses`);
      }
      if (listed.has(name)) {
        throw new Error(`display bindings list carrier '${name}' twice`);
      }
      listed.add(name);
    }
    for (const name of carrierWheels.keys()) {
      if (!listed.has(name)) {
        throw new Error(`carrier '${name}' has no display binding`);
      }
    }
  }

  const carbody: BoundBody | null =
    bindings === null || bindings.carbody === null
      ? null
      : {
          bodyName: bindings.carbody.bodyName,
          bodyIndex: requireBody(bindings.carbody.bodyName, 'the carbody binding'),
          displayName: bindings.carbody.displayName,
        };

  const bogieIndexOfBody = new Int32Array(record.bodies.length).fill(-1);
  const bogies: BoundBogie[] = (bindings?.bogies ?? []).map((bogie, bogieIndex) => {
    const what = `bogie binding '${bogie.bodyName}'`;
    const bodyIndex = requireBody(bogie.bodyName, what);
    const members = new Set<number>([bodyIndex]);
    for (const memberName of bogie.memberBodyNames) {
      members.add(requireBody(memberName, `${what} member list`));
    }
    // Wheels follow their carrier's group; the definition does not list them again.
    for (const wheels of carrierWheels.values()) {
      if (!members.has(wheels.bodyIndex)) {
        continue;
      }
      for (const placementIndex of [wheels.left, wheels.right]) {
        if (placementIndex !== null) {
          members.add(bodyIndexByName.get((record.wheelPlacements[placementIndex] as SceneRecord['wheelPlacements'][number]).wheelBodyName) as number);
        }
      }
    }
    const memberBodyIndices = [...members].sort((a, b) => a - b);
    for (const memberIndex of memberBodyIndices) {
      const existing = bogieIndexOfBody[memberIndex] as number;
      if (existing !== -1 && existing !== bogieIndex) {
        throw new Error(`body '${(record.bodies[memberIndex] as SceneRecord['bodies'][number]).name}' is listed under two bogies`);
      }
      bogieIndexOfBody[memberIndex] = bogieIndex;
    }
    return { bodyName: bogie.bodyName, bodyIndex, displayName: bogie.displayName, memberBodyIndices };
  });
  if (carbody !== null && bogieIndexOfBody[carbody.bodyIndex] !== -1) {
    throw new Error(`the carbody '${carbody.bodyName}' is listed as a bogie member`);
  }

  const carriers: BoundCarrier[] = carrierOrder.map(({ name, displayName }) => {
    const wheels = carrierWheels.get(name) as CarrierWheels;
    const bogieIndex = bogieIndexOfBody[wheels.bodyIndex] as number;
    return {
      bodyName: name,
      bodyIndex: wheels.bodyIndex,
      displayName,
      leftWheelPlacementIndex: wheels.left,
      rightWheelPlacementIndex: wheels.right,
      bogieIndex: bogieIndex === -1 ? null : bogieIndex,
    };
  });

  return { hasVisualDefinition: visualDefinition !== null, carbody, bogies, carriers, bogieIndexOfBody };
}
