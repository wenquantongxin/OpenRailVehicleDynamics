import type { BilingualTitle, PropertyReading } from '../service/vehicle_modeling_service_protocol.ts';

// How a subject's properties fall into sections, apart from the component so
// that the rule can be tested without a page: consecutive properties whose
// descriptions state the same section form one; a property without one stands
// in a section without a title.

export interface PropertySection {
  title: BilingualTitle | null;
  properties: PropertyReading[];
}

function sameSection(a: BilingualTitle | null, b: BilingualTitle | null): boolean {
  return a === null || b === null ? a === b : a.en === b.en && a.zh === b.zh;
}

export function propertySections(properties: readonly PropertyReading[]): PropertySection[] {
  const sections: PropertySection[] = [];
  for (const property of properties) {
    const last = sections.at(-1);
    if (last !== undefined && sameSection(last.title, property.section)) last.properties.push(property);
    else sections.push({ title: property.section, properties: [property] });
  }
  return sections;
}
