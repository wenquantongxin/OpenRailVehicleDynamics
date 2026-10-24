import type { SceneSampleView } from '../record/scene_sample.ts';
import type { ReadoutScales } from '../ui/record_summary.ts';
import { niceCeiling } from '../ui/format.ts';
import type { VehicleReadoutModel } from '../ui/vehicle_readout_model.ts';

/** Three maxima per run, without retaining a history of readouts. */
export class LiveReadoutScales {
  readonly scales: ReadoutScales = { forceKn: 10, bodyLateralMm: 1, bodyYawMrad: 1 };
  private readonly model: VehicleReadoutModel;
  constructor(model: VehicleReadoutModel) { this.model = model; }
  observe(sample: SceneSampleView): void {
    for (const wheel of this.model.wheels) {
      const force = this.model.read(sample, wheel.verticalSupportForceScalar);
      if (force?.valid) this.scales.forceKn = niceCeiling(Math.max(this.scales.forceKn, Math.abs(force.value) / 1000));
    }
    for (const body of [this.model.carbody, ...this.model.bogies]) {
      if (body === null) continue;
      const lateral = this.model.read(sample, body.lateralScalar);
      const yaw = this.model.read(sample, body.yawScalar);
      if (lateral?.valid) this.scales.bodyLateralMm = niceCeiling(Math.max(this.scales.bodyLateralMm, Math.abs(lateral.value) * 1000));
      if (yaw?.valid) this.scales.bodyYawMrad = niceCeiling(Math.max(this.scales.bodyYawMrad, Math.abs(yaw.value) * 1000));
    }
  }
}
