# The model-neutral description and executable assembly layer.
#
# G40 starts with a compiled description library rather than an INTERFACE
# target: construction validates a finalized model, so there is real code whose
# behavior must be compiled and tested.  The public contract names the existing
# first-party modelling facade and no vendored type.

add_library(orvd_system_assembly STATIC
    src/compiled_system_plan.cc
    src/system_assembly_description.cc
    src/system_instance.cc)

orvd_configure_first_party_target(orvd_system_assembly)

target_include_directories(orvd_system_assembly
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(orvd_system_assembly PUBLIC orvd_multibody_model)

# Deliberately not installed or exported.  Packaging is G46's concern; doing it
# here would freeze an assembly interface before it has an executable consumer.
