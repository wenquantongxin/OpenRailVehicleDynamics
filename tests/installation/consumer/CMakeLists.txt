cmake_minimum_required(VERSION 3.24)

project(orvd_installation_consumer LANGUAGES CXX)

find_package(OpenRailVehicleDynamics CONFIG REQUIRED)

if(TARGET nlohmann_json::nlohmann_json)
    message(FATAL_ERROR
        "the installed ORVD package unexpectedly imports nlohmann/json")
endif()
get_target_property(orvd_configuration_links
    ORVD::configuration INTERFACE_LINK_LIBRARIES)
if("${orvd_configuration_links}" MATCHES "nlohmann")
    message(FATAL_ERROR
        "ORVD::configuration leaks nlohmann/json: ${orvd_configuration_links}")
endif()

add_executable(orvd_installation_smoke installation_smoke.cc)
target_compile_features(orvd_installation_smoke PRIVATE cxx_std_23)
target_link_libraries(orvd_installation_smoke
    PRIVATE ORVD::integrators ORVD::track_geometry)

# Keep the strict configuration target isolated from every other ORVD target.
# This prevents another target from accidentally supplying a missing public
# include or link dependency and hiding a broken installed export.
add_executable(configuration_installation_smoke
    configuration_installation_smoke.cc)
target_compile_features(configuration_installation_smoke PRIVATE cxx_std_23)
target_link_libraries(configuration_installation_smoke
    PRIVATE ORVD::configuration)

# Keep a separate consumer that receives no include paths or transitive
# dependencies from another ORVD target. The combined smoke above executes the
# line calculation; this target independently proves that track_geometry alone
# carries its installed public-header and Eigen closure.
add_executable(track_geometry_installation_smoke
    track_geometry_installation_smoke.cc)
target_compile_features(track_geometry_installation_smoke PRIVATE cxx_std_23)
target_link_libraries(track_geometry_installation_smoke
    PRIVATE ORVD::track_geometry)

# This pair is a positive control for the installed-artifact dependency gate.
# The executable calls an exported function, so linkers cannot discard the
# Drake-identified shared library as unused.
add_library(drake_marker SHARED drake_marker.cc)
target_compile_features(drake_marker PRIVATE cxx_std_23)
target_compile_definitions(drake_marker PRIVATE ORVD_DRAKE_MARKER_BUILD)
set_target_properties(drake_marker PROPERTIES OUTPUT_NAME drake_marker)

add_executable(drake_loading_probe drake_loading_probe.cc)
target_compile_features(drake_loading_probe PRIVATE cxx_std_23)
target_link_libraries(drake_loading_probe PRIVATE drake_marker)
