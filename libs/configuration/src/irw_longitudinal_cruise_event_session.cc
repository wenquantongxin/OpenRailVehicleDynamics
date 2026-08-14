#include "orvd/configuration/irw_longitudinal_cruise_event_session.h"

#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

#include "irw_closed_topology.h"
#include "orvd/forces/independent_wheel_active_torque_plan.h"
#include "orvd/forces/wheel_rail_contact_force_plan.h"

namespace orvd::configuration {
namespace {

constexpr double kFrozenSamplePeriodSeconds = 0.01;

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("IRW longitudinal cruise event session: " +
                                detail);
}

[[nodiscard]] bool SameBits(double left, double right) {
    return std::bit_cast<std::uint64_t>(left) ==
           std::bit_cast<std::uint64_t>(right);
}

template <std::size_t Size>
void RequireFinite(const std::array<double, Size>& values,
                   const std::string& name) {
    for (const double value : values) {
        if (!std::isfinite(value)) {
            Reject(name + " contains a non-finite value");
        }
    }
}

}  // namespace

IrwLongitudinalCruiseEventSession::ResolvedBinding
IrwLongitudinalCruiseEventSession::ResolveBinding(
    const AssembledVehicleSystem& assembled) {
    const auto* contact_plan = assembled.contact_force_plan();
    const auto* torque_plan = assembled.active_torque_plan();
    if (contact_plan == nullptr || torque_plan == nullptr ||
        contact_plan->carrier_count() !=
            static_cast<int>(internal::kIrwClosedCarrierNames.size()) ||
        contact_plan->interface_count() !=
            static_cast<int>(internal::kIrwClosedInterfaces.size()) ||
        torque_plan->channel_count() !=
            static_cast<int>(internal::kIrwClosedInterfaces.size()) ||
        assembled.system().held_active_torque_count() !=
            static_cast<int>(internal::kIrwClosedInterfaces.size())) {
        Reject("the assembled system is not the closed four-carrier, "
               "eight-wheel IRW contact and active-torque topology");
    }
    for (std::size_t ordinal = 0;
         ordinal < internal::kIrwClosedCarrierNames.size(); ++ordinal) {
        if (contact_plan->carrier_name(static_cast<int>(ordinal)) !=
            internal::kIrwClosedCarrierNames[ordinal]) {
            Reject("the contact carrier order differs from the frozen IRW "
                   "cruise order");
        }
    }

    const auto& model = assembled.model();
    ResolvedBinding binding{
        .wheel_velocity_ranges = {
            model.GetJointVelocityRange(model.GetJointByName(
                internal::kIrwClosedInterfaces[0].revolute_joint_name)),
            model.GetJointVelocityRange(model.GetJointByName(
                internal::kIrwClosedInterfaces[1].revolute_joint_name)),
            model.GetJointVelocityRange(model.GetJointByName(
                internal::kIrwClosedInterfaces[2].revolute_joint_name)),
            model.GetJointVelocityRange(model.GetJointByName(
                internal::kIrwClosedInterfaces[3].revolute_joint_name)),
            model.GetJointVelocityRange(model.GetJointByName(
                internal::kIrwClosedInterfaces[4].revolute_joint_name)),
            model.GetJointVelocityRange(model.GetJointByName(
                internal::kIrwClosedInterfaces[5].revolute_joint_name)),
            model.GetJointVelocityRange(model.GetJointByName(
                internal::kIrwClosedInterfaces[6].revolute_joint_name)),
            model.GetJointVelocityRange(model.GetJointByName(
                internal::kIrwClosedInterfaces[7].revolute_joint_name)),
        },
    };
    for (std::size_t ordinal = 0;
         ordinal < internal::kIrwClosedInterfaces.size(); ++ordinal) {
        const auto& expected = internal::kIrwClosedInterfaces[ordinal];
        const int index = static_cast<int>(ordinal);
        if (contact_plan->interface_name(index) != expected.interface_name ||
            torque_plan->channel_name(index) != expected.interface_name ||
            torque_plan->axis_provider_body_name(index) !=
                expected.carrier_body_name ||
            torque_plan->wheel_body_name(index) !=
                expected.wheel_body_name ||
            torque_plan->reaction_frame_body_name(index) !=
                expected.reaction_frame_body_name) {
            Reject("contact, active torque and cruise channel " +
                   std::to_string(ordinal) +
                   " do not share the frozen IRW identity");
        }
        if (binding.wheel_velocity_ranges[ordinal].size() != 1) {
            Reject("an independent-wheel joint does not own exactly one "
                   "generalized velocity");
        }
    }
    return binding;
}

IrwLongitudinalCruiseEventSession::IrwLongitudinalCruiseEventSession(
    const AssembledVehicleSystem& assembled,
    control::SampledLongitudinalCruiseController controller,
    double nominal_rolling_radius_meters,
    actuation::WheelDriveTorqueChannelValues forward_joint_rate_signs,
    actuation::WheelDriveTorqueCommandConditioner conditioner)
    : IrwLongitudinalCruiseEventSession(
          assembled, std::move(controller), nominal_rolling_radius_meters,
          std::move(forward_joint_rate_signs), std::move(conditioner),
          ResolveBinding(assembled)) {}

IrwLongitudinalCruiseEventSession::IrwLongitudinalCruiseEventSession(
    const AssembledVehicleSystem& assembled,
    control::SampledLongitudinalCruiseController controller,
    double nominal_rolling_radius_meters,
    actuation::WheelDriveTorqueChannelValues forward_joint_rate_signs,
    actuation::WheelDriveTorqueCommandConditioner conditioner,
    ResolvedBinding binding)
    : assembled_(&assembled),
      controller_(std::move(controller)),
      conditioner_(std::move(conditioner)),
      wheel_velocity_ranges_(binding.wheel_velocity_ranges),
      forward_joint_rate_signs_(std::move(forward_joint_rate_signs)),
      nominal_rolling_radius_meters_(nominal_rolling_radius_meters),
      sample_period_seconds_(controller_.config().sample_period_seconds) {
    if (!std::isfinite(nominal_rolling_radius_meters_) ||
        !(nominal_rolling_radius_meters_ > 0.0)) {
        Reject("nominal_rolling_radius_meters must be finite and positive");
    }
    for (const double sign : forward_joint_rate_signs_) {
        if (sign != -1.0 && sign != 1.0) {
            Reject("forward joint-rate signs must be -1 or +1");
        }
    }
    if (!SameBits(sample_period_seconds_,
                  conditioner_.config().sample_period_seconds) ||
        !SameBits(sample_period_seconds_, kFrozenSamplePeriodSeconds)) {
        Reject("the controller and conditioner sample periods must be "
               "bitwise-equal frozen 0.01-second values");
    }
    if (conditioner_.config().forward_wheel_angular_speed_sign != -1.0) {
        Reject("the conditioner does not use the frozen negative-forward "
               "wheel-speed scalar convention");
    }
}

double IrwLongitudinalCruiseEventSession::
    next_periodic_event_time_seconds() const noexcept {
    return static_cast<double>(next_periodic_event_ordinal_) *
           sample_period_seconds_;
}

IrwLongitudinalCruiseWheelSpeedObservation
IrwLongitudinalCruiseEventSession::ObserveWheelSpeeds(
    const system_assembly::SystemRuntimeContext& context) const {
    IrwLongitudinalCruiseWheelSpeedObservation observation;
    const Eigen::VectorXd& velocities = context.generalized_velocities();
    if (velocities.size() != assembled_->model().num_generalized_velocities()) {
        Reject("the runtime context is not compatible with the assembled "
               "IRW model");
    }
    double sum = 0.0;
    for (std::size_t wheel = 0; wheel < wheel_velocity_ranges_.size();
         ++wheel) {
        const double raw_rate =
            velocities[wheel_velocity_ranges_[wheel].start()];
        observation.raw_joint_rates_radians_per_second[wheel] = raw_rate;
        observation
            .conditioner_scalar_wheel_speeds_radians_per_second[wheel] =
            -raw_rate;
        const double forward_speed = forward_joint_rate_signs_[wheel] *
                                     raw_rate *
                                     nominal_rolling_radius_meters_;
        observation
            .forward_wheel_circumferential_speeds_meters_per_second[wheel] =
            forward_speed;
        sum += forward_speed;
    }
    RequireFinite(observation.raw_joint_rates_radians_per_second,
                  "raw wheel joint rates");
    RequireFinite(
        observation
            .forward_wheel_circumferential_speeds_meters_per_second,
        "forward wheel circumferential speeds");
    RequireFinite(
        observation.conditioner_scalar_wheel_speeds_radians_per_second,
        "conditioner wheel-speed scalars");
    observation.common_forward_wheel_circumferential_speed_meters_per_second =
        sum / static_cast<double>(wheel_velocity_ranges_.size());
    if (!std::isfinite(
            observation
                .common_forward_wheel_circumferential_speed_meters_per_second)) {
        Reject("the common forward wheel circumferential speed is not "
               "finite");
    }
    return observation;
}

IrwLongitudinalCruiseEventAudit
IrwLongitudinalCruiseEventSession::ComputeCandidate(
    IrwLongitudinalCruiseEventKind kind, std::uint64_t ordinal,
    double event_time_seconds,
    const system_assembly::SystemRuntimeContext& accepted_context) const {
    IrwLongitudinalCruiseEventAudit audit;
    audit.kind = kind;
    audit.periodic_event_ordinal = ordinal;
    audit.event_time_seconds = event_time_seconds;
    audit.wheel_speed_observation = ObserveWheelSpeeds(accepted_context);
    audit.controller_state_before = controller_state_;
    audit.conditioner_memory_before_newton_metres =
        conditioner_memory_newton_metres_;
    audit.controller_result = controller_.Step(
        audit.wheel_speed_observation
            .common_forward_wheel_circumferential_speed_meters_per_second,
        controller_state_);
    audit.requested_wheel_torques_newton_metres.fill(
        audit.controller_result
            .requested_common_wheel_torque_newton_metres);
    audit.conditioning_result = conditioner_.Step(
        audit.requested_wheel_torques_newton_metres,
        audit.wheel_speed_observation
            .conditioner_scalar_wheel_speeds_radians_per_second,
        conditioner_memory_newton_metres_);
    return audit;
}

IrwLongitudinalCruiseEventAudit
IrwLongitudinalCruiseEventSession::ApplyInitializationUpdate(
    system_assembly::SystemRuntimeContext& accepted_context) {
    if (phase_ != Phase::kInitializationRequired) {
        Reject("the initialization event has already been committed");
    }
    if (!SameBits(accepted_context.time_seconds(), 0.0)) {
        Reject("the initialization event requires accepted time zero");
    }
    IrwLongitudinalCruiseEventAudit audit = ComputeCandidate(
        IrwLongitudinalCruiseEventKind::kInitialization, 0, 0.0,
        accepted_context);
    assembled_->system().SetHeldIndependentWheelActiveTorques(
        accepted_context,
        audit.conditioning_result.actual_wheel_torques_newton_metres);
    controller_state_ = audit.controller_result.next_state;
    conditioner_memory_newton_metres_ =
        audit.conditioning_result.next_drive_side_torque_memory_newton_metres;
    next_periodic_event_ordinal_ = 1;
    phase_ = Phase::kSynchronizationRequired;
    return audit;
}

IrwLongitudinalCruiseEventAudit
IrwLongitudinalCruiseEventSession::ApplyPeriodicUpdate(
    system_assembly::SystemRuntimeContext& accepted_context) {
    if (phase_ == Phase::kInitializationRequired) {
        Reject("the initialization event has not been committed");
    }
    if (phase_ == Phase::kSynchronizationRequired) {
        Reject("the preceding event is committed but its backend is not "
               "synchronized");
    }
    const double expected_time = next_periodic_event_time_seconds();
    if (!SameBits(accepted_context.time_seconds(), expected_time)) {
        Reject("accepted time does not equal the next integer-grid event "
               "time");
    }
    phase_ = Phase::kPeriodicEventRequired;
    IrwLongitudinalCruiseEventAudit audit = ComputeCandidate(
        IrwLongitudinalCruiseEventKind::kPeriodic,
        next_periodic_event_ordinal_, expected_time, accepted_context);
    assembled_->system().SetHeldIndependentWheelActiveTorques(
        accepted_context,
        audit.conditioning_result.actual_wheel_torques_newton_metres);
    controller_state_ = audit.controller_result.next_state;
    conditioner_memory_newton_metres_ =
        audit.conditioning_result.next_drive_side_torque_memory_newton_metres;
    ++next_periodic_event_ordinal_;
    phase_ = Phase::kSynchronizationRequired;
    return audit;
}

bool IrwLongitudinalCruiseEventSession::synchronization_required() const
    noexcept {
    return phase_ == Phase::kSynchronizationRequired;
}

void IrwLongitudinalCruiseEventSession::ConfirmBackendSynchronized() {
    if (phase_ != Phase::kSynchronizationRequired) {
        Reject("there is no committed event awaiting backend "
               "synchronization");
    }
    phase_ = Phase::kReadyToAdvance;
}

void IrwLongitudinalCruiseEventSession::RequireReadyToAdvance() const {
    if (phase_ != Phase::kReadyToAdvance) {
        Reject("the next vehicle advance is blocked until the required "
               "cruise event and backend synchronization succeed");
    }
}

}  // namespace orvd::configuration
