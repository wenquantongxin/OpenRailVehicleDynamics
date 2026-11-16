import type { ReferenceVehicle } from '../service/fetch_reference_vehicles.ts';
import type { ElementReference } from '../service/vehicle_modeling_service_protocol.ts';
import type { OpenChoice, StartLink } from './vehicle_editing_session.ts';

// The link a page address carries, followed at the start only when nothing is
// open: ?vehicle=<name> opens that reference vehicle, otherwise
// ?project=<name> opens a saved project, and &select=<kind>:<name> then
// selects an element. The project's name is passed on as it is: whether the
// folder holds it, the service says.

/** The element of a `select` link; the kind runs up to the first colon. */
function linkedElement(link: string): ElementReference {
  const separator = link.indexOf(':');
  if (separator < 1) {
    throw new Error(`the select link '${link}' is not <kind>:<name>`);
  }
  return { kind: link.slice(0, separator), name: link.slice(separator + 1) };
}

/** The link of the page's query, or null without one; a reference vehicle the catalogue does not have is refused. */
export function readStartLinkFromPageAddress(query: URLSearchParams, vehicles: readonly ReferenceVehicle[]): StartLink | null {
  const vehicleName = query.get('vehicle');
  const projectName = query.get('project');
  let open: OpenChoice;
  if (vehicleName !== null) {
    const vehicle = vehicles.find((entry) => entry.name === vehicleName);
    if (vehicle === undefined) {
      throw new Error(`no reference vehicle is named '${vehicleName}'`);
    }
    open = { source: 'reference_vehicle', vehicle };
  } else if (projectName !== null) {
    open = { source: 'vehicle_project', name: projectName };
  } else {
    return null;
  }
  const select = query.get('select');
  return { open, select: select === null ? null : linkedElement(select) };
}
