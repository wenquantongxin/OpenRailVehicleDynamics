"""Closed identities for IRW passive multi-speed SIMPACK direct-SLV scenarios."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Scenario:
    identifier: str
    simpack_track: str
    track_geometry: str
    irregularity_identifier: str
    initial_speed_kilometres_per_hour: float
    duration_seconds: float
    output_frequency_hertz: float
    meet_output_points: bool
    maximum_comparison_track_station_meters: float
    orvd_qualification_scenario_identifier: str | None
    response_plot_title: str
    response_plot_file_stem: str


SCENARIOS = {
    value.identifier: value
    for value in (
        Scenario(
            "irw_r300_aar5_v60_passive",
            "$Trk_Curve_R300m_60kmph",
            "r300_centerline_superelevation_1100m.json",
            "aar5_irregularity",
            60.0,
            30.0,
            100.0,
            False,
            1100.0,
            "irw_r300_aar5_v60_passive",
            "IRW R300 + frozen AAR5 passive at 60 km/h",
            "irw_r300_frozen_aar5_60kmph_30s",
        ),
        Scenario(
            "irw_straight_aar5_v80_passive",
            "$Trk_AAR5_80kmph",
            "straight_level_1100m.json",
            "aar5_irregularity",
            80.0,
            30.0,
            100.0,
            False,
            1100.0,
            "irw_straight_aar5_v80_passive",
            "IRW straight + frozen AAR5 passive at 80 km/h",
            "irw_straight_frozen_aar5_80kmph_30s",
        ),
        Scenario(
            "irw_r600_aar5_v80_passive",
            "$Trk_Curve_R600m_80kmph",
            "r600_centerline_superelevation_1100m.json",
            "aar5_irregularity",
            80.0,
            30.0,
            100.0,
            False,
            1100.0,
            "irw_r600_aar5_v80_passive",
            "IRW R600 + frozen AAR5 passive at 80 km/h",
            "irw_r600_frozen_aar5_80kmph_30s",
        ),
        Scenario(
            "irw_straight_aar6_v120_passive",
            "$Trk_AAR6_120kmph",
            "straight_level_1100m.json",
            "aar6_irregularity",
            120.0,
            8.0,
            100.0,
            False,
            300.0,
            "irw_straight_aar6_v120_passive",
            "IRW straight + frozen AAR6 passive at 120 km/h",
            "irw_straight_frozen_aar6_120kmph_8s",
        ),
        Scenario(
            "irw_r1000_aar6_v120_passive",
            "$Trk_Curve_R1000m_120kmph",
            "r1000_centerline_superelevation_300m.json",
            "aar6_irregularity",
            120.0,
            8.0,
            100.0,
            True,
            300.0,
            "irw_r1000_aar6_v120_passive",
            "IRW R1000 + frozen AAR6 passive at 120 km/h",
            "irw_r1000_frozen_aar6_120kmph_8s",
        ),
        Scenario(
            "irw_r800_aar5_v100_passive",
            "$Trk_Curve_R800m_100kmph",
            "r800_centerline_superelevation_1100m.json",
            "aar5_irregularity",
            100.0,
            30.0,
            100.0,
            True,
            1100.0,
            "irw_r800_aar5_v100_passive",
            "IRW R800 + frozen AAR5 passive at 100 km/h",
            "irw_r800_frozen_aar5_100kmph_30s",
        ),
        Scenario(
            "irw_straight_aar6_v160_passive",
            "$Trk_AAR6_160kmph",
            "straight_level_1100m.json",
            "aar6_irregularity",
            160.0,
            6.0,
            100.0,
            True,
            300.0,
            "irw_straight_aar6_v160_passive",
            "IRW straight + frozen AAR6 passive at 160 km/h",
            "irw_straight_frozen_aar6_160kmph_6s",
        ),
        Scenario(
            "irw_straight_erri_low_v200_passive",
            "$Trk_ERRI_low_200kmph",
            "straight_level_1100m.json",
            "erri_low_irregularity",
            200.0,
            8.0,
            100.0,
            True,
            500.0,
            "irw_straight_erri_low_v200_passive",
            "IRW straight + frozen ERRI low passive at 200 km/h",
            "irw_straight_frozen_erri_low_200kmph_8s",
        ),
    )
}


def require_scenario(identifier: str) -> Scenario:
    try:
        return SCENARIOS[identifier]
    except KeyError as error:
        choices = ", ".join(SCENARIOS)
        raise ValueError(
            f"unknown scenario {identifier!r}; expected one of: {choices}"
        ) from error
