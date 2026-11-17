import type { ProjectStartingPoint } from '../service/fetch_project_starting_points.ts';
import type { BilingualTitle } from '../service/vehicle_modeling_service_protocol.ts';

// The starting points under their categories, as the start page lists them,
// apart from its component so that it can be tested without a page: the
// categories in the order they first appear, each with its starting points in
// the server's order. A category only helps to find a starting point; it is
// known by its title alone.

export interface StartingPointCategory {
  category: BilingualTitle;
  startingPoints: ProjectStartingPoint[];
}

export function startingPointsByCategory(startingPoints: readonly ProjectStartingPoint[]): StartingPointCategory[] {
  const categories: StartingPointCategory[] = [];
  for (const startingPoint of startingPoints) {
    const { en, zh } = startingPoint.category;
    const listed = categories.find(({ category }) => category.en === en && category.zh === zh);
    if (listed === undefined) categories.push({ category: startingPoint.category, startingPoints: [startingPoint] });
    else listed.startingPoints.push(startingPoint);
  }
  return categories;
}
