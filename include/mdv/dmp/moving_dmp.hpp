#ifndef MDV_MOVING_DMP_HPP
#define MDV_MOVING_DMP_HPP

#include "mdv/mesh/fwd.hpp"

namespace mdv {

struct MovingDmpParameters {
    long   dt_ms              = 10;
    double dmp_tau            = 1.0;
    double circle_radius      = 0.01;
    double linear_speed       = 0.1;
    long   num_centroid_steps = 100;
    long   print_every        = 500;
};

std::vector<mdv::mesh::Point> generate_trajectory(
        MovingDmpParameters   params,
        mesh::Mesh&      mesh,
        const mesh::Geodesic& centroid_path
);


}  // namespace mdv


#endif  // MDV_MOVING_DMP_HPP
