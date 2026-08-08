#include "orvd/wheel_rail_contact/wheel_rail_contact_model.h"

namespace orvd::wheel_rail_contact {
namespace {

// The material is set on the model, not on the two force laws, so that they
// cannot be given different steel. Whatever a sub-configuration carried is
// replaced here rather than checked, because a check would leave the caller
// with two places to keep in step.
NormalContactConfiguration WithMaterial(NormalContactConfiguration configuration,
                                        const ContactMaterial& material) {
    configuration.material = material;
    return configuration;
}

TangentialContactConfiguration WithMaterial(
    TangentialContactConfiguration configuration, const ContactMaterial& material) {
    configuration.material = material;
    return configuration;
}

}  // namespace

WheelRailContactModel::WheelRailContactModel(
    const ProfilePoints& wheel_profile, const ProfilePoints& rail_profile,
    WheelSide side, const WheelProfilePreprocessingConfiguration& preparation,
    const WheelRailContactConfiguration& configuration)
    : geometry_(wheel_profile, rail_profile, side, preparation,
                configuration.geometry),
      normal_law_(WithMaterial(configuration.normal, configuration.material)),
      creep_coefficients_(KalkerCoefficientTable::ForPoissonRatio(
          configuration.material.poisson_ratio,
          configuration.creep_coefficients_outside_table)),
      tangential_solver_(
          WithMaterial(configuration.tangential, configuration.material),
          creep_coefficients_),
      creepage_(configuration.creepage),
      friction_(configuration.friction) {}

void WheelRailContactModel::PrepareWorkspace(
    WheelRailContactWorkspace& workspace) const {
    geometry_.PrepareWorkspace(workspace.geometry_);
    tangential_solver_.PrepareWorkspace(workspace.tangential_);
}

WheelRailContactResult WheelRailContactModel::Evaluate(
    const WheelRailContactInput& input,
    WheelRailContactWorkspace& workspace) const {
    WheelRailContactResult result;
    PrepareWorkspace(workspace);
    const ContactPatchSet patches =
        geometry_.Solve(input.pose, workspace.geometry_);
    result.geometric_patch_count = patches.count;
    result.three_dimensional_length_resolution_count = patches.count;

    for (std::size_t slot = 0; slot < patches.count; ++slot) {
        const ContactPatch& patch = patches.patches[slot];

        // The rail material reference point, in the track frame. The geometry
        // gave it in the rail's own three coordinates; the rail's frame places
        // it. This is the point at which the wheel material velocity is formed,
        // not the point about which the eventual wrench is reduced.
        const Eigen::Vector3d rail_reference(patch.rail_reference_longitudinal_meters,
                                             patch.rail_reference_lateral_meters,
                                             patch.rail_reference_vertical_meters);
        const Eigen::Vector3d rail_material_point =
            input.rail_frame.origin_track_meters +
            input.rail_frame.rotation_track_from_profile * rail_reference;

        // How fast the wheel's material is moving there. The rail's material is
        // stationary by construction — the rail is a shape the line carries,
        // not a body with a state — so the relative motion is the wheel's
        // alone. Writing the subtraction out would be writing a subtraction of
        // zero; naming the assumption here is the honest alternative.
        ContactRelativeMotion motion;
        motion.relative_velocity_track_meters_per_second =
            input.wheel.origin_velocity_track_meters_per_second +
            input.wheel.angular_velocity_track_radians_per_second.cross(
                rail_material_point - input.wheel.origin_track_meters);
        motion.relative_spin_track_radians_per_second =
            input.wheel.angular_velocity_track_radians_per_second;
        motion.arc_rate_meters_per_second = input.wheel.arc_rate_meters_per_second;
        motion.wheel_pitch_rate_radians_per_second =
            input.wheel.wheel_pitch_rate_radians_per_second;

        // One frame, built once, used by all three of the closing speed, the
        // creepages and the wrench. Building it three times is how the three
        // end up disagreeing.
        //
        // This is where the roll transport's offset lands, and the only place
        // inside this model that it does. The comparison is exact so that a
        // suppressed transport is bit-for-bit no transport rather than the
        // addition of a zero, which is not the same thing for a negative zero.
        const double frame_angle =
            input.roll_transport.roll_offset_radians == 0.0
                ? patch.rail_slope_angle_radians
                : patch.rail_slope_angle_radians +
                      input.roll_transport.roll_offset_radians;
        const ContactFrame frame = MakeContactFrame(frame_angle);

        NormalContactGeometry normal_geometry;
        normal_geometry.cross_section_area_square_meters =
            patch.cross_section_area_square_meters;
        normal_geometry.arc_width_meters = patch.arc_width_meters;
        normal_geometry.longitudinal_length_meters = patch.longitudinal_length_meters;
        normal_geometry.vertical_penetration_meters = patch.vertical_penetration_meters;
        normal_geometry.rolling_radius_meters = patch.rolling_radius_meters;
        normal_geometry.common_normal_angle_radians =
            patch.common_normal_angle_radians;

        const double approach_speed = ComputeNormalApproachSpeed(motion, frame);
        const NormalContactResult normal =
            normal_law_.Solve(normal_geometry, approach_speed);
        if (normal.used_analytic_longitudinal_length_fallback) {
            ++result.analytic_longitudinal_length_fallback_count;
        }
        if (!normal.in_contact || !(normal.normal_force_newtons > 0.0)) {
            // Geometrically touching but carrying nothing — a patch separating
            // fast enough that the damping has cancelled its elastic force.
            // There is no tangential force without a normal one, so the patch
            // is dropped rather than emitted with zeros.
            continue;
        }

        const Creepages creepages = ComputeCreepages(
            motion, frame, patch.rolling_radius_meters, creepage_);
        const double friction = FrictionCoefficientAt(friction_, creepages);

        TangentialContactPatch tangential_patch;
        tangential_patch.normal_force_newtons = normal.normal_force_newtons;
        tangential_patch.longitudinal_semi_axis_meters =
            normal.longitudinal_semi_axis_meters;
        tangential_patch.lateral_semi_axis_meters = normal.lateral_semi_axis_meters;
        tangential_patch.friction_coefficient = friction;
        const TangentialContactResult tangential = tangential_solver_.Solve(
            tangential_patch, creepages, workspace.tangential_);

        // The force on the wheel, in the contact's own axes. The normal
        // component is negative: the vertical points down, so a contact that
        // pushes the wheel up pushes it in the negative direction.
        const Eigen::Vector3d force_contact(tangential.longitudinal_force_newtons,
                                            tangential.lateral_force_newtons,
                                            -normal.normal_force_newtons);
        const Eigen::Vector3d force_track =
            frame.rotation_track_from_contact * force_contact;

        // The force acts at the compliant wheel-surface point, not at the rail
        // material reference point at which the relative velocity was formed.
        // The profile frame's origin already contains the wheelset-to-profile
        // lateral datum, so the patch's own authored station is used directly.
        // Half the equivalent penetration places the force at the middle of
        // the compliant approach, matching the qualified reference model's
        // p_GP construction.
        const Eigen::Vector3d wheel_surface_point_profile(
            patch.wheel_longitudinal_meters, patch.wheel_station_meters,
            patch.rolling_radius_meters -
                0.5 * normal.equivalent_penetration_meters);
        const Eigen::Vector3d wheel_surface_point =
            input.wheel.origin_track_meters +
            input.wheel.rotation_track_from_profile * wheel_surface_point_profile;

        WheelRailContactPatchResult& emitted = result.patches[result.count];
        // No moment accompanies the force at the contact point. The direct
        // moment a patch's creep would exert about its own normal is suppressed
        // in this formulation, so passing zero states that rather than omitting
        // it. Every moment the vehicle sees from this contact is the force's
        // moment about wherever it is transported to.
        emitted.wrench = MakePairedContactWrench(wheel_surface_point, force_track,
                                                 Eigen::Vector3d::Zero());
        emitted.geometry = patch;
        emitted.normal = normal;
        emitted.creepages = creepages;
        emitted.tangential = tangential;
        emitted.friction_coefficient = friction;
        emitted.approach_speed_meters_per_second = approach_speed;
        emitted.contact_frame_angle_radians = frame_angle;
        ++result.count;
    }
    return result;
}

}  // namespace orvd::wheel_rail_contact
