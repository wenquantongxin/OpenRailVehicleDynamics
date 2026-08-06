#include <cstdio>
#include <exception>

#include <Eigen/Core>

#include "orvd/wheel_rail_contact/contact_wrench.h"

int main() {
    try {
        const Eigen::Vector3d point(1.0, 2.0, 3.0);
        const Eigen::Vector3d force(4.0, 5.0, 6.0);
        const auto pair = orvd::wheel_rail_contact::MakePairedContactWrench(
            point, force, Eigen::Vector3d::Zero());
        if (pair.contact_point_meters != point ||
            pair.rail_on_wheel.force_newtons != force ||
            pair.wheel_on_rail.force_newtons != -force) {
            std::fprintf(stderr,
                         "installed wheel-rail contact wrench entry point "
                         "returned an inconsistent pair\n");
            return 1;
        }
    } catch (const std::exception& error) {
        std::fprintf(stderr, "installed wheel-rail contact smoke failed: %s\n",
                     error.what());
        return 1;
    }
    return 0;
}
