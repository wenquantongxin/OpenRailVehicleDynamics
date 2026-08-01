cmake_minimum_required(VERSION 3.24)

project(orvd_installation_consumer LANGUAGES CXX)

find_package(OpenRailVehicleDynamics CONFIG REQUIRED)

add_executable(orvd_installation_smoke installation_smoke.cc)
target_compile_features(orvd_installation_smoke PRIVATE cxx_std_23)
target_link_libraries(orvd_installation_smoke PRIVATE ORVD::integrators)

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
