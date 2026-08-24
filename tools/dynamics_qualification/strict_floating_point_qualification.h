#pragma once

// Source-private compile-time half of the qualification floating-point gate.
// CMake rejects known unsafe external tokens; these checks verify the final
// compiler semantics seen by each artifact writer.

#include <limits>
#include <string_view>

#if defined(__FAST_MATH__)
#error "ORVD strict floating-point qualification forbids __FAST_MATH__"
#endif

#if defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__ != 0
#error "ORVD strict floating-point qualification forbids finite-math-only semantics"
#endif

#if defined(__GNUC__) && !defined(__clang__) && \
    defined(__GCC_IEC_559) && __GCC_IEC_559 < 1
#error "ORVD strict floating-point qualification requires GCC IEC-559 semantics"
#endif

#ifndef ORVD_STRICT_FLOATING_POINT_FLAG_AUDIT_PASSED
#error "ORVD strict floating-point qualification requires the CMake flag audit"
#endif

#ifndef ORVD_STRICT_FLOATING_POINT_COMPILE_COMMAND_AUDIT_PASSED
#error "ORVD strict floating-point qualification requires proof from the final compile-command audit"
#endif

#ifndef ORVD_QUALIFICATION_CXX_COMPILER_ID
#error "ORVD qualification compiler identity is missing"
#endif

#ifndef ORVD_QUALIFICATION_CXX_COMPILER_VERSION
#error "ORVD qualification compiler version is missing"
#endif

#ifndef ORVD_QUALIFICATION_BUILD_TYPE
#error "ORVD qualification build type is missing"
#endif

namespace orvd::dynamics_qualification::internal {

inline constexpr std::string_view
    kStrictFloatingPointSemanticsIdentifier =
        "orvd.strict_ieee_no_fast_math.v1";
inline constexpr std::string_view kQualificationCxxCompilerId =
    ORVD_QUALIFICATION_CXX_COMPILER_ID;
inline constexpr std::string_view kQualificationCxxCompilerVersion =
    ORVD_QUALIFICATION_CXX_COMPILER_VERSION;
inline constexpr std::string_view kQualificationBuildType =
    ORVD_QUALIFICATION_BUILD_TYPE;

static_assert(std::numeric_limits<double>::is_iec559);
static_assert(std::numeric_limits<double>::radix == 2);
static_assert(std::numeric_limits<double>::digits == 53);
static_assert(std::numeric_limits<double>::max_exponent == 1024);

}  // namespace orvd::dynamics_qualification::internal
