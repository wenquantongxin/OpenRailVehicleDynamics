# Model-neutral construction of finite station-domain realizations from the
# simplified one-sided FRA/AAR spectra.  This layer owns neither asset parsing
# nor a vehicle scenario: its output is an ordinary pair of sampled fields that
# a higher layer may retain directly or turn into its interpolation type.

add_library(orvd_track_irregularity STATIC
    src/aar_track_irregularity_generator.cc)

orvd_configure_first_party_target(orvd_track_irregularity)

target_include_directories(orvd_track_irregularity
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

set_target_properties(orvd_track_irregularity PROPERTIES
    EXPORT_NAME track_irregularity)
add_library(ORVD::track_irregularity ALIAS orvd_track_irregularity)
