# The resident command layer a graphical modelling tool drives: newline-delimited
# JSON requests on standard input, one response line each on standard output.
# The protocol's JSON lives here; the libraries it calls show none in their
# headers.

add_executable(orvd_vehicle_modeling_service vehicle_modeling_service.cc)
orvd_configure_first_party_target(orvd_vehicle_modeling_service)
target_link_libraries(orvd_vehicle_modeling_service
    PRIVATE orvd_vehicle_project_editing nlohmann_json::nlohmann_json)
