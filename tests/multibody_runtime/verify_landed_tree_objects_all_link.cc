// Every object of the landed tree, forced into one link.
//
// This program does almost nothing on purpose. What is under test is not its
// behaviour but whether the link succeeds: it consumes the object libraries
// directly, so every translation unit of the landed tree is in the link whether
// this program refers to it or not.
//
// An ordinary consumer of the product archive cannot show this. A static archive
// hands over only the members that resolve a symbol the consumer already needs,
// so a translation unit with an unresolved symbol nobody happens to call sits in
// the archive undisturbed — until some later consumer calls it, at which point
// the failure surfaces a long way from what caused it.
//
// The mechanism is CMake's object libraries, not a linker option. `--whole-archive`
// is GNU-specific, and a check that only fires under one toolchain is not a check
// under the others.
#include <cstdio>

#include "drake/multibody/tree/multibody_tree-inl.h"

int main() {
    // Enough to instantiate the tree and prove it is the linked one rather than
    // a declaration nobody defined. The link itself is the assertion; this only
    // has to reach real code.
    //
    // Finalized before it is asked anything: the model refuses to report its
    // dimensions before then, and rightly — the answer does not exist yet.
    drake::multibody::internal::MultibodyTree<double> tree;
    tree.Finalize();
    if (tree.num_positions() != 0 || tree.num_velocities() != 0) {
        std::printf(
            "FAIL a model with only the world reports %d positions and %d "
            "velocities\n",
            tree.num_positions(), tree.num_velocities());
        return 1;
    }
    std::printf(
        "every landed translation unit is in this link and has no unresolved"
        " symbol\n");
    return 0;
}
