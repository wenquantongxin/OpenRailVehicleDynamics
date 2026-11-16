# The resident command layer a graphical modelling tool drives: newline-delimited
# JSON requests on standard input, one response line each on standard output.
# All JSON lives here; the editing library it calls holds none.

add_executable(orvd_vehicle_modeling_service vehicle_modeling_service.cc)
orvd_configure_first_party_target(orvd_vehicle_modeling_service)
target_link_libraries(orvd_vehicle_modeling_service
    PRIVATE orvd_vehicle_project_editing nlohmann_json::nlohmann_json)
