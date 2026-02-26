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
        mesh::Mesh&           mesh,
        const mesh::Geodesic& centroid_path
);

std::vector<mdv::mesh::Point> upsample_to_1khz(
        const mesh::Mesh&               mesh,
        const std::vector<mesh::Point>& in_path,
        MovingDmpParameters             params
);

std::vector<Eigen::Quaterniond> encode_orientation(
        const std::vector<mesh::Point>& in_path, bool flip_orientation = false
);

std::vector<Eigen::Quaterniond> filter_orientation(
        const std::vector<Eigen::Quaterniond>& qin, std::size_t window_size
);


}  // namespace mdv


#endif  // MDV_MOVING_DMP_HPP
