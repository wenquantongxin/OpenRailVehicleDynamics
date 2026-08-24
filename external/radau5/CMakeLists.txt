add_library(orvd_radau5_core_objects OBJECT
    src/radau5_core.cc)

orvd_configure_first_party_target(orvd_radau5_core_objects)

target_include_directories(orvd_radau5_core_objects
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(orvd_radau5_core_objects PUBLIC Eigen3::Eigen)
