#pragma once

#include <cstddef>
#include <span>
#include <string>
#include <utility>
#include <vector>

// A wheel or rail profile as a list of measured points, owned.
//
// The point list is the asset. Everything else in this module — splines,
// preprocessing, contact geometry — is derived from it, so this is the one
// place where the question "which points, in which frame, in which order" gets
// an answer, and every consumer downstream is spared having to ask again.
//
// Two stages exist and both are real. `ProfilePoints` holds the list exactly as
// the asset author wrote it, in the asset's own order; `SideResolvedProfile`
// holds it mirrored for the wheel it belongs to and sorted ascending in the
// lateral coordinate. The distinction is not bookkeeping. One preprocessing
// strategy marches a fixed step from the authored first element, so it sees a
// different grid depending on whether the asset was written flange-first or
// field-first; the other runs on the physical right-hand ordering and mirrors
// afterwards. An implementation that collapses the two stages into one gets the
// right answer for the right wheel and a silently different left wheel, and the
// two then stop being each other's mirror.
//
// The coordinates are metres in the profile's own frame: lateral across the
// track, vertical positive downward, consistent with the inertial frame this
// project uses everywhere. A profile carries no station, no side and no
// vehicle; it is a shape, and where that shape sits is somebody else's field.

namespace orvd::wheel_rail_contact {

// Which of the two surfaces the point list describes. The role is part of the
// asset's identity rather than a hint: a rail profile handed to a wheel
// consumer is a mistake worth refusing, not a coordinate convention to guess.
enum class ProfileRole {
    kWheel,
    kRail,
};

// Which wheel of a wheelset a profile is being resolved for. The track frame's
// lateral axis points right, so the right wheel keeps the authored sign and the
// left wheel is the mirror image.
enum class WheelSide {
    kRight,
    kLeft,
};

class SideResolvedProfile;

class ProfilePoints {
   public:
    // Builds the value object from the authored point list.
    //
    // The order is preserved exactly, including a descending list: the order is
    // information one preprocessing strategy consumes. What is checked is that
    // the points are usable at all — at least two of them, every coordinate
    // finite, and a lateral coordinate that runs strictly in one direction.
    // Sorting a list that doubles back would turn a malformed profile into a
    // plausible but different surface; a repeated abscissa is likewise
    // ill-posed for every interpolant on this list.
    //
    // Throws std::invalid_argument when the two spans differ in length, when
    // there are fewer than two points, when a coordinate is not finite, when a
    // lateral coordinate repeats or reverses direction, or when the identifier
    // is empty.
    [[nodiscard]] static ProfilePoints FromAuthoredOrder(
        ProfileRole role, std::string identifier,
        std::vector<double> lateral_meters, std::vector<double> vertical_meters);

    [[nodiscard]] ProfileRole role() const { return role_; }

    // The name this asset is known by, without a file extension. It is an
    // identity, not a path: what file it came from is the caller's business and
    // changes with the installation.
    [[nodiscard]] const std::string& identifier() const & { return identifier_; }
    [[nodiscard]] std::string identifier() && { return std::move(identifier_); }
    [[nodiscard]] std::string identifier() const && { return identifier_; }

    [[nodiscard]] std::span<const double> authored_lateral_meters() const & {
        return lateral_meters_;
    }
    [[nodiscard]] std::vector<double> authored_lateral_meters() && {
        return std::move(lateral_meters_);
    }
    [[nodiscard]] std::vector<double> authored_lateral_meters() const && {
        return lateral_meters_;
    }
    [[nodiscard]] std::span<const double> authored_vertical_meters() const & {
        return vertical_meters_;
    }
    [[nodiscard]] std::vector<double> authored_vertical_meters() && {
        return std::move(vertical_meters_);
    }
    [[nodiscard]] std::vector<double> authored_vertical_meters() const && {
        return vertical_meters_;
    }
    [[nodiscard]] std::size_t size() const { return lateral_meters_.size(); }

    // Mirrors for the given side and sorts ascending in the lateral coordinate.
    // The right side keeps every authored sign; only the order can change.
    [[nodiscard]] SideResolvedProfile ResolveForSide(WheelSide side) const;

   private:
    ProfilePoints(ProfileRole role, std::string identifier,
                  std::vector<double> lateral_meters,
                  std::vector<double> vertical_meters);

    ProfileRole role_{ProfileRole::kWheel};
    std::string identifier_;
    std::vector<double> lateral_meters_;
    std::vector<double> vertical_meters_;
};

// The same points, mirrored for one wheel and sorted ascending. Every
// interpolant in this module is built on one of these, never on the authored
// list, because an interpolant needs a strictly increasing abscissa and the
// authored list is not required to have one.
class SideResolvedProfile {
   public:
    [[nodiscard]] ProfileRole role() const { return role_; }
    [[nodiscard]] WheelSide side() const { return side_; }
    [[nodiscard]] std::span<const double> lateral_meters() const & {
        return lateral_meters_;
    }
    [[nodiscard]] std::vector<double> lateral_meters() && {
        return std::move(lateral_meters_);
    }
    [[nodiscard]] std::vector<double> lateral_meters() const && {
        return lateral_meters_;
    }
    [[nodiscard]] std::span<const double> vertical_meters() const & {
        return vertical_meters_;
    }
    [[nodiscard]] std::vector<double> vertical_meters() && {
        return std::move(vertical_meters_);
    }
    [[nodiscard]] std::vector<double> vertical_meters() const && {
        return vertical_meters_;
    }
    [[nodiscard]] std::size_t size() const { return lateral_meters_.size(); }

   private:
    friend class ProfilePoints;

    SideResolvedProfile(ProfileRole role, WheelSide side,
                        std::vector<double> lateral_meters,
                        std::vector<double> vertical_meters);

    ProfileRole role_{ProfileRole::kWheel};
    WheelSide side_{WheelSide::kRight};
    std::vector<double> lateral_meters_;
    std::vector<double> vertical_meters_;
};

}  // namespace orvd::wheel_rail_contact
