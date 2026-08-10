# Stateful actuator-command laws whose state remains owned by the caller.
#
# This layer does not know about vehicle topology, multibody forces, control
# algorithms or event scheduling.  It turns one command sample and one previous
# accepted-event memory value into a conditioned command and the next memory.

add_library(orvd_actuation STATIC
    src/wheel_drive_torque_command_conditioner.cc)

orvd_configure_first_party_target(orvd_actuation)

target_include_directories(orvd_actuation
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

set_target_properties(orvd_actuation PROPERTIES
    EXPORT_NAME actuation)
add_library(ORVD::actuation ALIAS orvd_actuation)
