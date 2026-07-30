// Asserts the index and ordering contract the vendored topology library provides.
//
// This is a structural behaviour contract, not a comparison against the
// installed Drake. The topology algorithms remain upstream implementations;
// G13 only removed unadmitted Graphviz APIs and repaired the resulting
// documentation. Comparing the same algorithms would not establish correctness.
//
// What is worth pinning down now is the contract every later pass will be built
// on: which link becomes which mobilized body, where each body's generalized
// coordinates start, how long they are, which tree a body belongs to, and the
// order the forest walks. Those are exact integers, so they are compared
// exactly — a tolerance here would only make a wrong answer survivable.
//
// Deliberately not frozen: subtree sizes, inboard coordinate counts and the
// other internal bookkeeping the forest keeps. Nothing in this goal depends on
// them, and pinning a number nobody reads turns an upstream refactor into a
// false failure.
#include <cstdio>
#include <string>
#include <string_view>
#include <vector>

#include "drake/multibody/topology/forest.h"
#include "drake/multibody/topology/graph.h"

namespace {

using drake::multibody::LinkIndex;
using drake::multibody::LinkOrdinal;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::internal::LinkJointGraph;
using drake::multibody::internal::SpanningForest;

int failure_count = 0;

void RecordFailureUnless(bool condition, std::string_view failure_description) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %.*s\n",
                     static_cast<int>(failure_description.size()),
                     failure_description.data());
        ++failure_count;
    }
}

template <typename Actual, typename Expected>
void ExpectEqual(std::string_view what, Actual actual, Expected expected) {
    const long long actual_value = static_cast<long long>(actual);
    const long long expected_value = static_cast<long long>(expected);
    if (actual_value != expected_value) {
        std::fprintf(stderr, "FAILED: %.*s: got %lld, expected %lld\n",
                     static_cast<int>(what.size()), what.data(), actual_value,
                     expected_value);
        ++failure_count;
    }
}

// What a single mobilized body must look like. Only the fields this contract
// covers appear here.
struct ExpectedMobod {
    int mobod_index;
    int link_index;      // persistent LinkIndex
    int link_ordinal;    // position in the graph's link vector; may be renumbered
    int tree_index;      // -1 for World, which belongs to no tree
    int level;
    int position_start;
    int position_count;
    int velocity_start;
    int velocity_count;
    bool is_reversed;  // ignored for World, where this flag has no meaning
};

LinkJointGraph MakeGraphWithSingleDofJointTypes() {
    LinkJointGraph graph;
    graph.RegisterJointType("revolute", 1, 1);
    graph.RegisterJointType("prismatic", 1, 1);
    return graph;
}

void CheckForest(std::string_view scenario, LinkJointGraph* graph,
                 int expected_position_count, int expected_velocity_count,
                 int expected_tree_count,
                 const std::vector<ExpectedMobod>& expected_mobods) {
    if (!graph->BuildForest()) {
        RecordFailureUnless(false, std::string(scenario) + ": BuildForest failed");
        return;
    }
    const SpanningForest& forest = graph->forest();
    const std::string prefix(scenario);

    ExpectEqual(prefix + ": num_positions", forest.num_positions(),
                expected_position_count);
    ExpectEqual(prefix + ": num_velocities", forest.num_velocities(),
                expected_velocity_count);
    ExpectEqual(prefix + ": tree count", forest.trees().size(), expected_tree_count);
    ExpectEqual(prefix + ": mobod count", forest.mobods().size(),
                expected_mobods.size());
    if (forest.mobods().size() != expected_mobods.size()) return;

    for (const ExpectedMobod& expected : expected_mobods) {
        const SpanningForest::Mobod& mobod =
            forest.mobods()[static_cast<std::size_t>(expected.mobod_index)];
        const std::string at =
            prefix + ": mobod[" + std::to_string(expected.mobod_index) + "]";

        ExpectEqual(at + " index", mobod.index(), expected.mobod_index);

        // Exercise both public link identities and their round trip. The current
        // graph API does not remove links, so these scenarios do not claim to
        // distinguish equal-valued indexes and ordinals.
        const LinkOrdinal link_ordinal = mobod.active_link_ordinal();
        ExpectEqual(at + " link ordinal", link_ordinal, expected.link_ordinal);
        const LinkIndex link_index = graph->links(link_ordinal).index();
        ExpectEqual(at + " link index", link_index, expected.link_index);
        ExpectEqual(at + " link index round trip",
                    graph->link_by_index(link_index).ordinal(),
                    expected.link_ordinal);
        ExpectEqual(at + " link-to-mobod mapping",
                    graph->link_to_mobod(link_index), expected.mobod_index);

        ExpectEqual(at + " tree",
                    mobod.tree().is_valid() ? static_cast<int>(mobod.tree()) : -1,
                    expected.tree_index);
        ExpectEqual(at + " level", mobod.level(), expected.level);
        ExpectEqual(at + " q_start", mobod.q_start(), expected.position_start);
        ExpectEqual(at + " nq", mobod.nq(), expected.position_count);
        ExpectEqual(at + " v_start", mobod.v_start(), expected.velocity_start);
        ExpectEqual(at + " nv", mobod.nv(), expected.velocity_count);
        // Drake documents the value for World as not meaningful.
        if (expected.mobod_index != 0) {
            ExpectEqual(at + " is_reversed", mobod.is_reversed() ? 1 : 0,
                        expected.is_reversed ? 1 : 0);
        }
    }
}

// World is mobod 0 at level 0 with no coordinates, in every scenario.
constexpr ExpectedMobod kWorldMobod{0, 0, 0, -1, 0, 0, 0, 0, 0, false};

void CheckSerialChain() {
    LinkJointGraph graph = MakeGraphWithSingleDofJointTypes();
    graph.AddLink("first", ModelInstanceIndex(1));
    graph.AddLink("second", ModelInstanceIndex(1));
    graph.AddLink("third", ModelInstanceIndex(1));
    graph.AddJoint("world_to_first", ModelInstanceIndex(1), "revolute",
                   LinkIndex(0), LinkIndex(1));
    graph.AddJoint("first_to_second", ModelInstanceIndex(1), "revolute",
                   LinkIndex(1), LinkIndex(2));
    graph.AddJoint("second_to_third", ModelInstanceIndex(1), "revolute",
                   LinkIndex(2), LinkIndex(3));
    CheckForest("serial chain", &graph, 3, 3, 1,
                {kWorldMobod,
                 {1, 1, 1, 0, 1, 0, 1, 0, 1, false},
                 {2, 2, 2, 0, 2, 1, 1, 1, 1, false},
                 {3, 3, 3, 0, 3, 2, 1, 2, 1, false}});
}

void CheckBranchedTreeIsWalkedDepthFirst() {
    // Two branches off `shoulder`. The deeper branch is registered before the
    // shallower one is finished, so mobod numbering can only come out as
    // 1,2,4,3 in link-ordinal terms if the forest is walked depth first.
    // Breadth first would number the two level-2 bodies consecutively and give
    // 1,2,3,4 instead — which is what makes this a real test of the order and
    // not a restatement of it.
    LinkJointGraph graph = MakeGraphWithSingleDofJointTypes();
    graph.AddLink("shoulder", ModelInstanceIndex(1));      // LinkIndex 1
    graph.AddLink("deep_branch", ModelInstanceIndex(1));   // LinkIndex 2
    graph.AddLink("shallow_branch", ModelInstanceIndex(1));  // LinkIndex 3
    graph.AddLink("deep_branch_tip", ModelInstanceIndex(1));  // LinkIndex 4
    graph.AddJoint("world_to_shoulder", ModelInstanceIndex(1), "revolute",
                   LinkIndex(0), LinkIndex(1));
    graph.AddJoint("shoulder_to_deep", ModelInstanceIndex(1), "revolute",
                   LinkIndex(1), LinkIndex(2));
    graph.AddJoint("deep_to_tip", ModelInstanceIndex(1), "revolute", LinkIndex(2),
                   LinkIndex(4));
    graph.AddJoint("shoulder_to_shallow", ModelInstanceIndex(1), "revolute",
                   LinkIndex(1), LinkIndex(3));

    CheckForest("branched tree", &graph, 4, 4, 1,
                {kWorldMobod,
                 {1, 1, 1, 0, 1, 0, 1, 0, 1, false},
                 // Depth first: the deep branch is finished to its tip before
                 // the shallow branch is visited at all.
                 {2, 2, 2, 0, 2, 1, 1, 1, 1, false},
                 {3, 4, 4, 0, 3, 2, 1, 2, 1, false},
                 {4, 3, 3, 0, 2, 3, 1, 3, 1, false}});
}

void CheckWeldConsumesNoCoordinates() {
    LinkJointGraph graph = MakeGraphWithSingleDofJointTypes();
    graph.AddLink("hinged", ModelInstanceIndex(1));
    graph.AddLink("welded_to_hinged", ModelInstanceIndex(1));
    graph.AddJoint("world_to_hinged", ModelInstanceIndex(1), "revolute",
                   LinkIndex(0), LinkIndex(1));
    graph.AddJoint("weld_joint", ModelInstanceIndex(1), "weld", LinkIndex(1),
                   LinkIndex(2));
    // The welded body still gets a coordinate start — one past the end of the
    // hinge's — but zero length. A pass that treated a zero-length slice as
    // absent would read the hinge's coordinate instead.
    CheckForest("weld", &graph, 1, 1, 1,
                {kWorldMobod,
                 {1, 1, 1, 0, 1, 0, 1, 0, 1, false},
                 {2, 2, 2, 0, 2, 1, 0, 1, 0, false}});
    RecordFailureUnless(graph.forest().mobods()[2].is_weld(),
                        "weld: the welded mobod must report itself as a weld");
}

void CheckFreeBodyGetsSevenPositionsAndSixVelocities() {
    LinkJointGraph graph = MakeGraphWithSingleDofJointTypes();
    graph.AddLink("unattached", ModelInstanceIndex(1));
    // No joint at all: the forest inserts a quaternion floating joint, whose
    // four quaternion components make the position count differ from the
    // velocity count. That asymmetry is the reason q and v are indexed
    // separately everywhere downstream.
    CheckForest("free body", &graph, 7, 6, 1,
                {kWorldMobod, {1, 1, 1, 0, 1, 0, 7, 0, 6, false}});
}

void CheckMixedJointsSplitIntoSeparateTrees() {
    LinkJointGraph graph = MakeGraphWithSingleDofJointTypes();
    graph.AddLink("hinged", ModelInstanceIndex(1));
    graph.AddLink("sliding", ModelInstanceIndex(1));
    graph.AddLink("welded", ModelInstanceIndex(1));
    graph.AddLink("unattached", ModelInstanceIndex(1));
    graph.AddJoint("world_to_hinged", ModelInstanceIndex(1), "revolute",
                   LinkIndex(0), LinkIndex(1));
    graph.AddJoint("hinged_to_sliding", ModelInstanceIndex(1), "prismatic",
                   LinkIndex(1), LinkIndex(2));
    graph.AddJoint("sliding_to_welded", ModelInstanceIndex(1), "weld",
                   LinkIndex(2), LinkIndex(3));
    // The unattached body is its own tree, and its coordinates continue after
    // the chain's rather than being interleaved with them.
    CheckForest("mixed joints", &graph, 9, 8, 2,
                {kWorldMobod,
                 {1, 1, 1, 0, 1, 0, 1, 0, 1, false},
                 {2, 2, 2, 0, 2, 1, 1, 1, 1, false},
                 {3, 3, 3, 0, 3, 2, 0, 2, 0, false},
                 {4, 4, 4, 1, 1, 2, 7, 2, 6, false}});
}

void CheckJointTowardsWorldProducesReversedMobilizer() {
    // The joint is declared with World as its child, so the forest has to grow
    // through it from the child side. That is the only way is_reversed() becomes
    // true, and every scenario above leaves it false — without this case the
    // flag would go untested by the whole topology contract.
    LinkJointGraph graph = MakeGraphWithSingleDofJointTypes();
    graph.AddLink("attached_backwards", ModelInstanceIndex(1));
    graph.AddJoint("towards_world", ModelInstanceIndex(1), "revolute",
                   LinkIndex(1), LinkIndex(0));
    CheckForest("reversed mobilizer", &graph, 1, 1, 1,
                {kWorldMobod, {1, 1, 1, 0, 1, 0, 1, 0, 1, true}});
}

}  // namespace

int main() {
    CheckSerialChain();
    CheckBranchedTreeIsWalkedDepthFirst();
    CheckWeldConsumesNoCoordinates();
    CheckFreeBodyGetsSevenPositionsAndSixVelocities();
    CheckMixedJointsSplitIntoSeparateTrees();
    CheckJointTowardsWorldProducesReversedMobilizer();

    if (failure_count > 0) {
        std::fprintf(stderr, "%d topology index check(s) failed\n", failure_count);
        return 1;
    }
    std::printf("vendored topology index assignment verified\n");
    return 0;
}
