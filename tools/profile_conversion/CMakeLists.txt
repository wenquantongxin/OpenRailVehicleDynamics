# Local profile interoperability, for research work between this project and the
# reference multibody tool.
#
# Deliberately outside the product module list: nothing here is installed,
# nothing here is exported, and no product target links it. The direction of
# use runs the other way — this reads the reference format and writes the
# project's own record, so that the runtime never has to.

add_library(orvd_profile_conversion STATIC
    profile_json_writer.cc
    simpack_profile_io.cc)

orvd_configure_first_party_target(orvd_profile_conversion)

target_include_directories(orvd_profile_conversion
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(orvd_profile_conversion PUBLIC orvd_wheel_rail_contact)

add_executable(convert_profile convert_profile.cc)
orvd_configure_first_party_target(convert_profile)
target_link_libraries(convert_profile
    PRIVATE orvd_profile_conversion orvd_configuration)
