#pragma once

/// @file
/// A forward declaration of the rigid tree's evaluation context.
///
/// Most of the tree only ever holds a reference to a context and passes it
/// along. Those declarations need the name and nothing else, and including the
/// full definition for them would put every cache workspace type into the
/// include graph of files that never touch one — which is how a header that
/// nobody depends on in spirit becomes one that everybody depends on in fact.
///
/// The definition lives next to this file. Include it where a member is actually
/// reached for, and this one everywhere else.

namespace orvd::rigid_multibody_tree::internal {

class RigidMultibodyTreeEvaluationContext;

}  // namespace orvd::rigid_multibody_tree::internal
