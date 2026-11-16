# The command layer a graphical modelling tool drives over one vehicle project:
# every element's properties described and read, edits validated at this
# boundary with undo and redo, a mechanical check and the preview a 3D view
# draws. It holds no JSON and reads and writes no files.

add_library(orvd_vehicle_project_editing STATIC
    src/element_properties.cc
    src/vehicle_project_editing_session.cc
    src/vehicle_project_preview_computation.cc)

orvd_configure_first_party_target(orvd_vehicle_project_editing)

target_include_directories(orvd_vehicle_project_editing
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(orvd_vehicle_project_editing
    PUBLIC orvd_vehicle_project)

set_target_properties(orvd_vehicle_project_editing PROPERTIES
    EXPORT_NAME vehicle_project_editing)
add_library(ORVD::vehicle_project_editing ALIAS orvd_vehicle_project_editing)
