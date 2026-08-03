#include <cmath>
#include <cstdio>
#include <exception>
#include <stdexcept>

#include <Eigen/Core>

#include "orvd/configuration/load_track_geometry.h"

int main(int argc, char* argv[]) {
    try {
        if (argc != 2) {
            throw std::invalid_argument(
                "expected the installed track geometry path");
        }
        const auto line =
            orvd::configuration::LoadTrackGeometryFromJsonFile(argv[1]);
        const auto point = line.CenterlinePositionInInertialMeters(40.0);
        if (std::abs(point.x() - 40.0) > 1.0e-12 || point.y() != 0.0 ||
            point.z() != 0.0) {
            std::fprintf(stderr,
                         "installed configuration smoke produced an invalid "
                         "straight-line point\n");
            return 1;
        }
    } catch (const std::exception& error) {
        std::fprintf(stderr, "installed configuration smoke failed: %s\n",
                     error.what());
        return 1;
    }
    return 0;
}
