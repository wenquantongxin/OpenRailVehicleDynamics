#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>

#include "orvd/configuration/load_track_irregularity_field.h"
#include "orvd/configuration/resolve_track_irregularity_field.h"
#include "orvd/track_irregularity/aar_track_irregularity_generator.h"

namespace {

using orvd::configuration::FrozenTrackIrregularityFieldSource;
using orvd::configuration::GeneratedAarTrackIrregularityFieldSource;
using orvd::configuration::LoadTrackIrregularityFieldFromDataRoot;
using orvd::configuration::ResolveTrackIrregularityField;
using orvd::configuration::TrackIrregularityFieldSource;
using orvd::track_irregularity::AarTrackClass;
using orvd::track_irregularity::AarTrackIrregularityGenerationSpec;
using orvd::track_irregularity::DeriveAarTrackIrregularityChannelSeeds;
using orvd::track_irregularity::GenerateAarTrackIrregularity;

int failures = 0;

void Require(bool condition, std::string_view message) {
    if (!condition) {
        std::fprintf(stderr, "track-irregularity source resolution: %.*s\n",
                     static_cast<int>(message.size()), message.data());
        ++failures;
    }
}

AarTrackIrregularityGenerationSpec MakeGeneratedSpecification(
    std::uint64_t realization_seed) {
    return AarTrackIrregularityGenerationSpec{
        AarTrackClass::kAar5,
        {0.05, 0.25, 41},
        {0.0, 100.0, 0.1},
        {10.0, 90.0, 10.0, 10.0},
        realization_seed};
}

void CheckFrozenSource(const std::filesystem::path& data_root) {
    struct FrozenFixture {
        std::string_view identifier;
        std::array<double, 3> comparison_stations_meters;
    };
    constexpr std::array<FrozenFixture, 3> fixtures{{
        {"aar5_irregularity", {160.1, 450.0, 1099.9}},
        {"aar6_irregularity", {50.1, 150.0, 299.9}},
        {"erri_low_irregularity", {50.1, 250.0, 499.9}},
    }};

    for (const FrozenFixture& fixture : fixtures) {
        const auto resolved = ResolveTrackIrregularityField(
            data_root,
            TrackIrregularityFieldSource{FrozenTrackIrregularityFieldSource{
                std::string(fixture.identifier)}});
        Require(resolved.field != nullptr &&
                    !resolved.generated_metadata.has_value(),
                "an authorized frozen source did not resolve unchanged");
        if (!resolved.field) {
            continue;
        }
        const auto directly_loaded = LoadTrackIrregularityFieldFromDataRoot(
            data_root, fixture.identifier);
        for (const double station : fixture.comparison_stations_meters) {
            Require(
                resolved.field->LateralDisplacementMeters(station) ==
                        directly_loaded.LateralDisplacementMeters(station) &&
                    resolved.field->VerticalDisplacementMeters(station) ==
                        directly_loaded.VerticalDisplacementMeters(station) &&
                    resolved.field->LateralSlopeMetersPerMeter(station) ==
                        directly_loaded.LateralSlopeMetersPerMeter(station) &&
                    resolved.field->VerticalSlopeMetersPerMeter(station) ==
                        directly_loaded.VerticalSlopeMetersPerMeter(station),
                "a frozen source transformed the strict loader field");
        }
    }

    bool rejected_unknown_identifier = false;
    try {
        static_cast<void>(ResolveTrackIrregularityField(
            data_root,
            TrackIrregularityFieldSource{FrozenTrackIrregularityFieldSource{
                "unlisted_irregularity"}}));
    } catch (const std::invalid_argument&) {
        rejected_unknown_identifier = true;
    }
    Require(rejected_unknown_identifier,
            "the closed frozen source accepted an unlisted field");
}

void CheckGeneratedSource() {
    constexpr std::uint64_t kRealizationSeed = 0x0123456789abcdefULL;
    const AarTrackIrregularityGenerationSpec specification =
        MakeGeneratedSpecification(kRealizationSeed);
    const auto resolved = ResolveTrackIrregularityField(
        {}, TrackIrregularityFieldSource{
                GeneratedAarTrackIrregularityFieldSource{specification}});
    Require(resolved.field != nullptr,
            "the generated source returned no field");
    Require(resolved.generated_metadata.has_value(),
            "the generated source omitted its metadata");
    if (!resolved.field || !resolved.generated_metadata) {
        return;
    }

    const auto expected_channel_seeds =
        DeriveAarTrackIrregularityChannelSeeds(kRealizationSeed);
    const auto& metadata = *resolved.generated_metadata;
    Require(metadata.specification.realization_seed == kRealizationSeed &&
                metadata.lateral.seed == expected_channel_seeds.lateral &&
                metadata.vertical.seed == expected_channel_seeds.vertical &&
                metadata.lateral.seed != metadata.vertical.seed,
            "the generated field did not retain its seed identity");

    for (const double station : {9.95, 9.999, 90.001, 90.05}) {
        Require(resolved.field->LateralDisplacementMeters(station) == 0.0 &&
                    resolved.field->VerticalDisplacementMeters(station) ==
                        0.0 &&
                    resolved.field->LateralSlopeMetersPerMeter(station) ==
                        0.0 &&
                    resolved.field->VerticalSlopeMetersPerMeter(station) ==
                        0.0,
                "a generated field was nonzero outside placement");
    }
    Require(resolved.field->LateralDisplacementMeters(10.0) == 0.0 &&
                resolved.field->VerticalDisplacementMeters(10.0) == 0.0 &&
                resolved.field->LateralDisplacementMeters(90.0) == 0.0 &&
                resolved.field->VerticalDisplacementMeters(90.0) == 0.0,
            "a generated field was nonzero at a placement endpoint");

    const auto generated = GenerateAarTrackIrregularity(specification);
    constexpr std::array<std::size_t, 2> kFadeProbeIndices{101, 899};
    constexpr double kSplineKnotToleranceMeters = 1.0e-15;
    for (const std::size_t sample_index : kFadeProbeIndices) {
        const double station =
            generated.track_station_meters[sample_index];
        Require(
            std::abs(resolved.field->LateralDisplacementMeters(station) -
                     generated.lateral_displacement_meters[sample_index]) <=
                    kSplineKnotToleranceMeters &&
                std::abs(resolved.field->VerticalDisplacementMeters(station) -
                         generated.vertical_displacement_meters[sample_index]) <=
                    kSplineKnotToleranceMeters,
            "the generated source did not preserve both placement fades");
    }

    const double lateral_midpoint =
        resolved.field->LateralDisplacementMeters(50.0);
    const double vertical_midpoint =
        resolved.field->VerticalDisplacementMeters(50.0);
    Require(std::isfinite(lateral_midpoint) &&
                std::isfinite(vertical_midpoint) &&
                (std::abs(lateral_midpoint) > 1.0e-12 ||
                 std::abs(vertical_midpoint) > 1.0e-12),
            "the generated field had no finite full-amplitude excitation");

    const auto repeated = ResolveTrackIrregularityField(
        {}, TrackIrregularityFieldSource{
                GeneratedAarTrackIrregularityFieldSource{specification}});
    Require(repeated.field->LateralDisplacementMeters(50.0) ==
                    lateral_midpoint &&
                repeated.field->VerticalDisplacementMeters(50.0) ==
                    vertical_midpoint,
            "the same specification and seed did not replay the field");

    const auto changed = ResolveTrackIrregularityField(
        {}, TrackIrregularityFieldSource{
                GeneratedAarTrackIrregularityFieldSource{
                    MakeGeneratedSpecification(kRealizationSeed + 1)}});
    Require(changed.field->LateralDisplacementMeters(50.0) !=
                    lateral_midpoint ||
                changed.field->VerticalDisplacementMeters(50.0) !=
                    vertical_midpoint,
            "a different realization seed repeated both channels");
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: %s <ORVD data root>\n", argv[0]);
        return 2;
    }
    CheckFrozenSource(argv[1]);
    CheckGeneratedSource();
    return failures == 0 ? 0 : 1;
}
