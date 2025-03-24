#ifndef MDV_MESH_UV_MAP_HPP
#define MDV_MESH_UV_MAP_HPP

#include <Eigen/Dense>

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"

namespace mdv::mesh {

//  _   _       __  __
// | | | |_   _|  \/  | __ _ _ __
// | | | \ \ / / |\/| |/ _` | '_ \
// | |_| |\ V /| |  | | (_| | |_) |
//  \___/  \_/ |_|  |_|\__,_| .__/
//                          |_|
class UvMap {
public:
    using Domain    = Eigen::Vector2d;
    using CoDomain  = Point3d;
    using Transform = Eigen::Matrix<double, 3, 2>;

    UvMap() : _origin(CoDomain::Zero()), _transform(Transform::Zero()) {};

    UvMap(const CoDomain& v1, const CoDomain& v2, const CoDomain& v3) : _origin(v1) {
        assert(v1 != v2);
        assert(v1 != v3);
        assert(v2 != v3);
        _transform.col(0) = v2 - v1;
        _transform.col(1) = v3 - v1;
    }

    MDV_NODISCARD CoDomain
    forward_map(const Domain& in) const {
        return _origin + _transform * in;
    };

    MDV_NODISCARD
    const Transform&
    forward_map_jacobian() const {
        return _transform;
    }

    MDV_NODISCARD Domain
    inverse_map(const CoDomain& in) const {
        return _transform.householderQr().solve(in - _origin);
    }

private:
    CoDomain  _origin;
    Transform _transform;
};


}  // namespace mdv::mesh


#endif  // MDV_MESH_UV_MAP_HPP
