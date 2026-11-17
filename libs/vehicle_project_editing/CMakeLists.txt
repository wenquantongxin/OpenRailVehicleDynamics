# The command layer a graphical modelling tool drives over one vehicle project:
# the model tree by domains, the properties of every element and of the
# project's settings described and read, how each rigid body is attached read
# and changed, elements created, edited, renamed and removed and settings
# edited at this boundary with undo and redo, saving to a project file, a
# mechanical check and the preview a 3D view draws. It holds no JSON.
# The check reaches the configuration library's declaration rules through its
# internal directory, which is not installed.

add_library(orvd_vehicle_project_editing STATIC
    src/vehicle_project_editing_session.cc
    src/vehicle_project_element_creation.cc
    src/vehicle_project_element_name_spaces.cc
    src/vehicle_project_element_property_assignment.cc
    src/vehicle_project_element_references.cc
    src/vehicle_project_element_removal.cc
    src/vehicle_project_model_tree_by_domains.cc
    src/vehicle_project_preview_computation.cc
    src/vehicle_project_property_edit_value_checks.cc
    src/vehicle_project_property_subjects.cc
    src/vehicle_project_records_following_elements.cc
    src/vehicle_project_rigid_body_attachment_changes.cc
    src/vehicle_project_rigid_body_attachments.cc
    src/vehicle_project_settings_property_assignment.cc
    src/vehicle_project_subject_properties.cc)

orvd_configure_first_party_target(orvd_vehicle_project_editing)

target_include_directories(orvd_vehicle_project_editing
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/libs/configuration/internal>)

target_link_libraries(orvd_vehicle_project_editing
    PUBLIC orvd_vehicle_project)

set_target_properties(orvd_vehicle_project_editing PROPERTIES
    EXPORT_NAME vehicle_project_editing)
add_library(ORVD::vehicle_project_editing ALIAS orvd_vehicle_project_editing)
