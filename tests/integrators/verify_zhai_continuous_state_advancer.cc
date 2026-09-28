#include "basic_coordinate_advancer_contract.h"

int main() {
    using namespace orvd::integrators::internal;
    return test::RunBasicCoordinateAdvancerContract<ZhaiContinuousStateAdvancer>();
}
