#pragma once

/// @file
/// Writes one portable scene record directory.
///
/// The directory holds:
///   scene.json               public description, static assets and the
///                            frame and scalar status table layouts;
///   frames.f64le             one row per frame of little-endian binary64:
///                            [time_seconds, time_nanoseconds, sample_index,
///                            phase, 13 values per body in slot order,
///                            one unwrapped spin angle per wheel when the
///                            record carries them, one value per scalar];
///                            integer identities are exact up to 2^53 and
///                            refused beyond;
///   scalar_statuses.u8       one byte per frame per scalar;
///   visual_definition.json   the parametric visual definition, copied
///                            verbatim when the caller supplies one.
///
/// A record without scene.json is incomplete: `Close()` writes it last, so a
/// run that stops early leaves nothing a reader accepts. The record promises
/// only what was exported; it does not carry the continuous state, the
/// projection history or anything a reader could use to re-evaluate physics.

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <vector>

#include "orvd/scene_observation/scene_description.h"
#include "orvd/scene_observation/scene_frame.h"
#include "orvd/scene_record/scene_assets.h"

namespace orvd::scene_record {

class SceneRecordWriter {
   public:
    /// Creates `directory`, which must not exist yet. A non-empty
    /// `assets.visual_definition_json` must be one JSON object and is copied
    /// verbatim; the writer does not interpret it.
    /// `with_wheel_spin_angles` states whether every frame carries one
    /// unwrapped spin angle per wheel placement; a frame must then supply
    /// exactly that many, and none otherwise.
    SceneRecordWriter(
        const std::filesystem::path& directory,
        scene_observation::SceneDescription description,
        SceneAssets assets, bool with_wheel_spin_angles);
    ~SceneRecordWriter();

    SceneRecordWriter(const SceneRecordWriter&) = delete;
    SceneRecordWriter& operator=(const SceneRecordWriter&) = delete;
    SceneRecordWriter(SceneRecordWriter&&) = delete;
    SceneRecordWriter& operator=(SceneRecordWriter&&) = delete;

    /// Appends one frame. The body count must equal the topology's and the
    /// scalar count the definitions'; a scalar whose status is not valid must
    /// carry the value zero; every number must be finite.
    void WriteFrame(const scene_observation::SceneFrame& frame);

    /// Stores the sampled line once, before Close(); assets must not already
    /// contain a track table.
    void WriteTrackSampleTable(
        const scene_observation::TrackSampleTable& table);

    /// Flushes the tables and writes scene.json. No write is admitted after.
    void Close();

    [[nodiscard]] std::size_t frame_count() const noexcept {
        return frame_count_;
    }
    [[nodiscard]] const std::filesystem::path& directory() const noexcept {
        return directory_;
    }

   private:
    std::filesystem::path directory_;
    scene_observation::SceneDescription description_;
    SceneAssets assets_;
    std::size_t wheel_spin_angle_count_{0};
    std::ofstream frames_;
    std::ofstream statuses_;
    std::vector<unsigned char> row_bytes_;
    std::size_t frame_count_{0};
    bool closed_{false};
};

}  // namespace orvd::scene_record
