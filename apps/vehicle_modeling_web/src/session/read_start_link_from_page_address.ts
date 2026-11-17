import type { ProjectStartingPoint } from '../service/fetch_project_starting_points.ts';
import type { ElementReference } from '../service/vehicle_modeling_service_protocol.ts';
import type { OpenChoice, StartLink } from './vehicle_editing_session.ts';

// The link a page address carries, followed at the start only when nothing is
// open: ?starting_point=<name> starts a new project from that starting point,
// otherwise ?project=<name> opens a saved project, and &select=<kind>:<name>
// then selects an element. The project's name is passed on as it is: whether
// the folder holds it, the service says.

/** The element of a `select` link; the kind runs up to the first colon. */
function linkedElement(link: string): ElementReference {
  const separator = link.indexOf(':');
  if (separator < 1) {
    throw new Error(`the select link '${link}' is not <kind>:<name>`);
  }
  return { kind: link.slice(0, separator), name: link.slice(separator + 1) };
}

/** The link of the page's query, or null without one; a starting point the server does not offer is refused. */
export function readStartLinkFromPageAddress(query: URLSearchParams, startingPoints: readonly ProjectStartingPoint[]): StartLink | null {
  const startingPointName = query.get('starting_point');
  const projectName = query.get('project');
  let open: OpenChoice;
  if (startingPointName !== null) {
    const startingPoint = startingPoints.find((entry) => entry.name === startingPointName);
    if (startingPoint === undefined) {
      throw new Error(`no starting point is named '${startingPointName}'`);
    }
    open = { source: 'starting_point', startingPoint };
  } else if (projectName !== null) {
    open = { source: 'vehicle_project', name: projectName };
  } else {
    return null;
  }
  const select = query.get('select');
  return { open, select: select === null ? null : linkedElement(select) };
}
