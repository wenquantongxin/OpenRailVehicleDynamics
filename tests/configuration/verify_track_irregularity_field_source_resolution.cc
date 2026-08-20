#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>

#include "orvd/configuration/resolve_track_irregularity_field.h"
#include "orvd/track_irregularity/aar_track_irregularity_generator.h"

namespace {

using orvd::configuration::FrozenTrackIrregularityFieldSource;
using orvd::configuration::GeneratedAarTrackIrregularityFieldSource;
using orvd::configuration::ResolveTrackIrregularityField;
using orvd::configuration::TrackIrregularityFieldSource;
using orvd::track_irregularity::AarTrackClass;
using orvd::track_irregularity::AarTrackIrregularityGenerationSpec;
using orvd::track_irregularity::DeriveAarTrackIrregularityChannelSeeds;

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
    const auto resolved = ResolveTrackIrregularityField(
        data_root,
        TrackIrregularityFieldSource{FrozenTrackIrregularityFieldSource{
            "aar5_irregularity"}});
    Require(resolved.field != nullptr,
            "the frozen source returned no field");
    Require(!resolved.generated_metadata.has_value(),
            "the frozen source claimed generated metadata");
    Require(resolved.field->LateralDisplacementMeters(-1.0) == 0.0 &&
                resolved.field->VerticalDisplacementMeters(1101.0) == 0.0,
            "the frozen field lost its existing outside-domain contract");
    Require(std::isfinite(
                resolved.field->LateralDisplacementMeters(450.0)) &&
                std::isfinite(
                    resolved.field->VerticalDisplacementMeters(450.0)),
            "the frozen field did not remain evaluable");

    for (const std::string_view identifier : {
             "aar6_irregularity", "erri_low_irregularity"}) {
        const auto another = ResolveTrackIrregularityField(
            data_root,
            TrackIrregularityFieldSource{FrozenTrackIrregularityFieldSource{
                std::string(identifier)}});
        Require(another.field != nullptr &&
                    !another.generated_metadata.has_value(),
                "an authorized frozen source did not resolve unchanged");
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
