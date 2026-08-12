#pragma once

#include <array>
#include <cstddef>
#include <span>
#include <vector>

#include "orvd/wheel_rail_contact/monotone_cubic_interpolant.h"
#include "orvd/wheel_rail_contact/natural_cubic_spline.h"
#include "orvd/wheel_rail_contact/profile_points.h"
#include "orvd/wheel_rail_contact/wheel_profile_preprocessing.h"
#include "orvd/wheel_rail_contact/wheel_rail_pose.h"

// Where a wheel and a rail overlap, and what shape the overlap has.
//
// This is the geometric half of the contact problem. It takes one pose — the
// four scalars the pose reduction produced — and returns the patches: for each,
// where it sits, how deep it is, how wide, how long, at what angle, and how
// curved the two surfaces are there. It computes no force. Everything a force
// law needs about the shape of the contact comes from here, and nothing that
// depends on material or load does.
//
// ## The three questions it answers, in order
//
// **Where is the wheel?** The wheel is a surface of revolution, so at a given
// yaw there is, for each transverse station across the profile, one
// circumferential angle at which the surface is furthest forward along the
// yawed running direction. Sweeping that angle across the profile traces the
// silhouette the rail can actually see. That silhouette is the *visible
// outline*, and projecting it into the rail's own cross-section plane is the
// first thing this does.
//
// **Where do the two surfaces overlap?** The projected outline is collapsed
// into a single-valued function of lateral position — an upper envelope — and
// differenced against the rail surface on a shared grid. The difference is a
// vertical interpenetration, positive where the two bodies overlap. Its
// positive runs are the contact islands.
//
// **What shape is each island?** Each island is integrated on its own uniform
// grid to give an area, a width, a centroid, the deepest penetration, and the
// angles and curvatures at the contact point.
//
// ## What is deliberately not here
//
// No force, no material, no load. No contact-patch pressure distribution — that
// belongs to the tangential solution. No track irregularity: the rail is where
// the pose said it is, and the pose is where the irregularity, when there is
// one, has already been accounted for.
//
// Several quantities the reference model computes are diagnostics that no force
// consumes, and they are not computed here: the penetration-weighted contact
// angle (which the reference computes on a second quadrature grid and then
// unconditionally overwrites with the common normal), the near-miss gap outside
// the islands, the per-island merge-gap bookkeeping, the two closed-form
// chord-length proxies that only non-selected length modes read, and the two
// alternative patch widths — the rail's arc across the patch and the wheel's
// arc taken with the longitudinal sweep included. The last of those is why
// there is no second interpolant over the projected outline's longitudinal
// coordinate: nothing that survives to a force reads it. Leaving these out is a
// scope decision, not an oversight; a consumer that needs one of them needs it
// added here rather than approximated elsewhere.
//
// ## Vertical sign
//
// Both profiles measure height positive *away from* the body — downward for the
// wheel sitting on the rail, upward-negative for the rail. So the overlap is
// wheel height minus rail height, positive in contact, and the "upper" envelope
// is the one with the largest such height: the outermost material, the part
// that can actually touch.

namespace orvd::wheel_rail_contact {

// How many islands the segmentation will track, and how many patches the solve
// will return.
//
// Both are ceilings on a physically bounded quantity: a wheel and a rail in one
// cross-section produce one contact patch normally, two through a flange
// contact, and three only in transient geometry. The limits are far above that
// so that a pathological pose is truncated rather than allowed to allocate.
inline constexpr std::size_t kMaxContactIslands = 64;
inline constexpr std::size_t kMaxContactPatches = 16;

struct ContactGeometryConfiguration {
    // The wheel's nominal rolling radius: the radius at the profile's own
    // vertical datum, from which every local radius is an excursion.
    double nominal_rolling_radius_meters{0.42};
    // The lateral resolution at which the projected outline is collapsed into a
    // single-valued function. Not a sampling step — nothing is evaluated at a
    // bin's centre. It is the width within which two projected points are
    // treated as competing for the same lateral position, so that a fold in the
    // projection keeps only its outermost branch.
    double envelope_bin_width_meters{2.0e-5};
    // How many stations the visible outline is traced at.
    std::size_t outline_sample_count{1000};
    // The overlap above which a point counts as in contact. Compared strictly,
    // so a gap of exactly zero is not contact.
    double contact_gap_epsilon_meters{0.0};
    // Two islands separated by a valley shallower than this are one island.
    // This is a vertical separation, not a lateral distance.
    double island_merge_gap_tolerance_meters{1.0e-6};
    // Stations per island for the area, width and centroid quadrature. This
    // grid is not a refinement of the grid the islands were found on: it is
    // usually coarser, and the deepest penetration it finds may legitimately
    // differ from the deepest the segmentation saw.
    std::size_t island_quadrature_stations{120};
    // The rail's laying cant, as the geometry solver carries it. This is added
    // to the rail's local surface slope to give the angle the contact and force
    // frames are built on, and it is a different constant from the cant the
    // pose reduction uses even when the two agree to eight decimals.
    double rail_cant_radians{0.024994792};
};

// One contact patch: everything about its shape that a force law reads.
struct ContactPatch {
    // The angle of the common normal — the rail's surface normal at the patch
    // centroid, carried into the wheelset's frame. It characterises the two
    // profile surfaces geometrically; the selected GZ18 force and creepage
    // frame below deliberately does not use it. The rail's laying cant is NOT
    // added to it: the cant is already inside the pose's roll.
    double common_normal_angle_radians{0.0};
    // The rail's own surface slope angle at the centroid, plus the laying cant.
    // A different angle from the one above, and the one the contact frame and
    // the force frame are both built on.
    double rail_slope_angle_radians{0.0};

    // Where the patch is, in the rail profile's lateral coordinate, and how
    // high the area centroid sits.
    double centroid_lateral_meters{0.0};
    double centroid_vertical_meters{0.0};

    // Where the patch is on the wheel's own profile, and the local rolling
    // radius there. The lateral coordinate carries no wheelset bias: it is
    // measured on the profile, and the distance out to the axle is the
    // consumer's to add.
    double wheel_station_meters{0.0};
    double wheel_longitudinal_meters{0.0};
    double rolling_radius_meters{0.0};

    // The deepest overlap, measured straight down the rail frame's vertical,
    // and the same depth turned onto the rail's surface normal.
    double vertical_penetration_meters{0.0};
    double normal_penetration_meters{0.0};

    // Three widths of the same island: the straight distance between its two
    // edges, and the arc length the wheel surface covers between them. The
    // second is what the contact ellipse's semi-axis is drawn from.
    double chord_width_meters{0.0};
    double arc_width_meters{0.0};

    // The overlap's cross-sectional area, plain and weighted by the wheel's arc
    // element.
    double cross_section_area_square_meters{0.0};
    double arc_weighted_area_square_meters{0.0};

    // How far the overlap extends along the running direction. Zero means the
    // three-dimensional resolution was attempted but could not bracket the
    // extent. It is an unavailable measurement, not a zero-length patch; the
    // normal law then uses its analytic longitudinal baseline and reports that
    // it did so.
    double longitudinal_length_meters{0.0};

    // The two surfaces' curvatures at the contact point. The wheel's is taken
    // on its own undeformed profile at the wheel station; the rail's is taken
    // in the rail frame at the centroid. They are deliberately not both in the
    // same frame.
    double wheel_curvature_per_meter{0.0};
    double rail_curvature_per_meter{0.0};

    // The point on the undeformed rail surface that this patch reduces about,
    // in the rail profile's own three coordinates. It is not the wheel point
    // and not a pressure centroid: it is where a rail material point is taken
    // to sit so that both bodies' velocities can be formed at one place.
    double rail_reference_longitudinal_meters{0.0};
    double rail_reference_lateral_meters{0.0};
    double rail_reference_vertical_meters{0.0};
};

struct ContactPatchSet {
    // Ascending in `centroid_lateral_meters`. A consumer that wants the patch
    // nearest the tread takes the one with the smallest normal angle, not the
    // first.
    std::array<ContactPatch, kMaxContactPatches> patches{};
    std::size_t count{0};
    // How many islands the raw segmentation found and how many survived the
    // merge. Kept because a patch count that changes between steps is almost
    // always a merge threshold doing something, and these two say so.
    std::size_t island_count{0};
    std::size_t merged_island_count{0};
};

// The buffers one solve needs, owned by the caller so that the solver itself
// stays immutable and two poses can be solved in any order.
//
// Every buffer is sized once, from the solver's own capacities, and never
// shrinks. A solve does not clear them: each stage writes its own prefix before
// reading it. Two contexts must not share one workspace.
class ContactGeometryWorkspace {
  public:
    ContactGeometryWorkspace() = default;

  private:
    friend class ContactGeometrySolver;

    void EnsureCapacity(std::size_t sample_capacity, std::size_t bin_capacity,
                        std::size_t envelope_capacity, std::size_t union_capacity,
                        std::size_t quadrature_capacity);

    // The projected outline, one entry per sampled station.
    std::vector<double> projected_lateral_;
    std::vector<double> projected_vertical_;
    std::vector<double> projected_station_;

    // The binning, one entry per bin.
    std::vector<double> bin_highest_;
    std::vector<int> bin_argument_;
    std::vector<std::size_t> bin_generation_;
    std::size_t current_bin_generation_{0};

    // The envelope, one entry per occupied bin.
    std::vector<double> envelope_lateral_;
    std::vector<double> envelope_vertical_;
    std::vector<double> envelope_station_;
    std::vector<double> envelope_vertical_slopes_;
    // The two per-interval arrays the shape-preserving slope rule needs. The
    // envelope is rebuilt every solve, so these are held rather than allocated.
    std::vector<double> envelope_spacing_scratch_;
    std::vector<double> envelope_secant_scratch_;

    // The shared grid the two surfaces are differenced on.
    std::vector<double> union_lateral_;
    std::vector<double> union_gap_;

    // The per-island quadrature. Every integral over an island is accumulated
    // in one sweep from these, so there is no buffer per integrand.
    std::vector<double> quadrature_lateral_;
    std::vector<double> quadrature_rail_;
    std::vector<double> quadrature_overlap_;
    std::vector<double> quadrature_wheel_slope_;
    std::vector<double> quadrature_station_;

    std::array<std::size_t, kMaxContactIslands> island_start_{};
    std::array<std::size_t, kMaxContactIslands> island_end_{};
    std::array<std::size_t, kMaxContactIslands> merged_start_{};
    std::array<std::size_t, kMaxContactIslands> merged_end_{};

    std::size_t envelope_size_{0};
    std::size_t union_size_{0};
};

class ContactGeometrySolver {
  public:
    // Builds the solver for one wheel of one wheelset against one rail.
    //
    // Both profiles are resolved for the given side here, so the caller hands
    // in the asset as authored. The wheel's preparation is applied at the same
    // time: whatever node set it lays is the node set every later evaluation of
    // the wheel surface uses.
    //
    // Throws std::invalid_argument when a profile has the wrong role, when
    // either has fewer than three points, or when a configuration number is not
    // usable.
    ContactGeometrySolver(const ProfilePoints& wheel_profile,
                          const ProfilePoints& rail_profile, WheelSide side,
                          const WheelProfilePreprocessingConfiguration& preparation,
                          const ContactGeometryConfiguration& configuration);

    // Solves one finite pose. Once PrepareWorkspace has sized the workspace,
    // no pose allocates. A failure to resolve a positive patch's
    // three-dimensional longitudinal extent is published as an unavailable
    // measurement; the normal law owns the analytic baseline used in that
    // case. Throws std::logic_error only if an implementation change violates
    // the solver's proven workspace bounds.
    [[nodiscard]] ContactPatchSet Solve(const ContactPoseScalars& pose,
                                        ContactGeometryWorkspace& workspace) const;

    // Sizes a workspace for this solver without solving.
    void PrepareWorkspace(ContactGeometryWorkspace& workspace) const;

    [[nodiscard]] const ContactGeometryConfiguration& configuration() const & {
        return configuration_;
    }
    [[nodiscard]] const ContactGeometryConfiguration& configuration() const && =
        delete;
    [[nodiscard]] WheelSide side() const { return side_; }

    // The wheel node set the preparation laid, and the outline traced over it.
    // Exposed so that a caller can check the preparation against the same
    // observable the preprocessing layer publishes rather than infer it.
    [[nodiscard]] std::span<const double> wheel_node_lateral_meters() const & {
        return wheel_nodes_.lateral_meters;
    }
    [[nodiscard]] std::span<const double> wheel_node_lateral_meters() const && =
        delete;
    [[nodiscard]] std::span<const double> outline_station_meters() const & {
        return outline_station_;
    }
    [[nodiscard]] std::span<const double> outline_station_meters() const && = delete;
    [[nodiscard]] std::span<const double> outline_height_meters() const & {
        return outline_height_;
    }
    [[nodiscard]] std::span<const double> outline_height_meters() const && = delete;
    [[nodiscard]] std::span<const double> outline_height_slope() const & {
        return outline_slope_;
    }
    [[nodiscard]] std::span<const double> outline_height_slope() const && = delete;

  private:
    // How far along the running direction the overlap actually extends.
    //
    // The envelope collapsed the wheel's circumferential degree of freedom, so
    // the patch's longitudinal extent is not in it. This restores it: at each
    // station across the patch it walks the wheel's own circle outward from the
    // angle the silhouette was taken at until the surface clears the rail on
    // both sides, then measures the chord between those two crossings. The
    // longest such chord over the patch is the answer.
    //
    // Returns exactly zero when no station could be bracketed. `Solve`
    // publishes that unavailable measurement so the normal law can apply and
    // report the analytic baseline without a second geometry evaluation.
    [[nodiscard]] double ResolveLongitudinalLength(
        const ContactPoseScalars& pose, double lateral_offset,
        double vertical_offset, std::span<const double> stations) const;

    ContactGeometryConfiguration configuration_;
    WheelSide side_{WheelSide::kRight};
    // +1 on the left, -1 on the right: the direction the laying cant tilts the
    // rail's surface angle for this side.
    double side_sign_{-1.0};

    // The node set the wheel's preparation laid, and the rail's own nodes. Both
    // are held rather than the profiles they came from: after this point the
    // authored assets are not consulted again.
    WheelProfileControlNodes wheel_nodes_;
    SideResolvedProfile rail_side_;

    // The wheel surface, twice. The spline is what the outline's slope comes
    // from and what the longitudinal resolution walks the wheel circle with;
    // the interpolant is what every profile query inside a patch uses. Inside
    // the node range they are the same curve — a cubic is fixed by its end
    // values and end slopes — and outside it they are not, which is why both
    // exist.
    NaturalCubicSpline wheel_spline_;
    MonotoneCubicInterpolant wheel_surface_;
    MonotoneCubicInterpolant rail_surface_;

    std::vector<double> outline_station_;
    std::vector<double> outline_height_;
    std::vector<double> outline_slope_;

    std::size_t bin_capacity_{0};
    std::size_t envelope_capacity_{0};
    std::size_t union_capacity_{0};
    std::size_t quadrature_capacity_{0};
};

}  // namespace orvd::wheel_rail_contact
