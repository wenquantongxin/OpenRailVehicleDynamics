#pragma once

/// @file
/// The revision counter of one source of multibody state.
///
/// A version records *how many write transactions this source has accepted*, not
/// what it currently holds. It is not a hash of the values: writing the same
/// numbers again is still a write, and it still advances. The alternative —
/// comparing the new value against the live one and skipping the advance when
/// they match — would put a full pass over the data on the write path and would
/// tie a freshness question to a value-equality question, which are not the same
/// question.
///
/// Advancing when nothing really changed costs a recomputation: the answer is
/// right and the work is wasted, and the waste is visible to a recompute count.
/// Failing to advance when something did change yields a stale answer: it is
/// wrong, and nothing reports it. The asymmetry decides the design.
///
/// The type is deliberately narrow. It compares for equality and it produces its
/// successor; it does not convert to an integer, does not do arithmetic, and does
/// not order. A cache asks "is this the same version I recorded?" — never "is it
/// newer?", which would invite a comparison across two different sources, and
/// never "how many versions apart?", which means nothing.

#include <cstdint>
#include <limits>
#include <stdexcept>

namespace orvd::multibody_runtime {

class MultibodyStateVersion {
   public:
    /// The version of a source that has accepted no write yet.
    ///
    /// This is only "nothing has been written". It is not a sentinel for "no
    /// snapshot has been taken" — that is a property of a cache slot, and a slot
    /// that borrowed this value would be indistinguishable from one that had
    /// legitimately recorded a freshly built state.
    constexpr MultibodyStateVersion() = default;

    /// The underlying count, for diagnostics and tests.
    ///
    /// Named rather than implicit: an implicit conversion would let a version be
    /// added, indexed with, or compared against an unrelated integer, and every
    /// one of those reads as sensible code.
    [[nodiscard]] constexpr std::uint64_t revision_number() const {
        return revision_number_;
    }

    /// The version that follows this one.
    ///
    /// @throws std::overflow_error at the ceiling. Wrapping would eventually make
    /// a recorded version match a state that had moved on, and a stale value
    /// would be served as fresh with nothing to indicate it. Refusing is the only
    /// behaviour that cannot be silently wrong.
    [[nodiscard]] MultibodyStateVersion Next() const {
        if (revision_number_ == std::numeric_limits<std::uint64_t>::max()) {
            throw std::overflow_error(
                "multibody state version: the revision counter is exhausted; "
                "advancing would wrap and make a stale value look fresh");
        }
        return MultibodyStateVersion(revision_number_ + 1);
    }

    /// Equality only. See the file comment for why there is no ordering.
    [[nodiscard]] constexpr bool operator==(
        const MultibodyStateVersion&) const = default;

   private:
    explicit constexpr MultibodyStateVersion(std::uint64_t revision_number)
        : revision_number_(revision_number) {}

    std::uint64_t revision_number_{0};
};

}  // namespace orvd::multibody_runtime
