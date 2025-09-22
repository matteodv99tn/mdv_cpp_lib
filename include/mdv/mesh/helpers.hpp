#ifndef MDV_MESH_HELPERS_HPP
#define MDV_MESH_HELPERS_HPP

#include <Eigen/Dense>

#include "mdv/macros.hpp"
#include "mdv/mesh/half_edge.hpp"
#include "mdv/utils/conditions.hpp"

namespace mdv::mesh {

class Edge {
public:
    // Implement affine types
    // struct Point3 : Eigen::Vector3d {};
    // struct Vec3 : Eigen::Vector3d {};

    using Point3 = Eigen::Vector3d;
    using Vec3   = Eigen::Vector3d;

    Edge(const HalfEdge& he) : _o(he.origin_position()), _d(he.direction()) {}

    static Edge
    from_position_and_direction(const Point3& pos, const Vec3& dir) {
        return {pos, dir};
    }

    static Edge
    from_positions(const Point3& p1, const Point3& p2) {
        return {p1, p2 - p1};
    }

    bool
    coplanar_with(const Edge& other) const {
        const Vec3 delta_os = this->_o - other._o;
        return mdv::condition::are_parallel(
                delta_os.cross(this->_d), delta_os.cross(other._d)
        );
    }

    MDV_NODISCARD const Point3&
    origin() const {
        return _o;
    }

    MDV_NODISCARD const Vec3&
    direction() const {
        return _d;
    }


private:
    Edge(const Point3& pos, const Vec3& dir) : _o(pos), _d(dir) {
        assert(!mdv::condition::is_zero_norm(_d));
    }

    Point3 _o;  // origin
    Vec3   _d;  // (non-normalised) direction
};

class EdgeIntersection {
public:
    using Mat32 = Eigen::Matrix<double, 3, 2>;
    using Vec2  = Eigen::Vector2d;
    using Vec3  = Eigen::Vector3d;

    EdgeIntersection(const Edge& e1, const Edge& e2) {
        assert(e1.coplanar_with(e2));

        Mat32 A;
        A.col(0)     = -e1.direction();
        A.col(1)     = e2.direction();
        const Vec3 b = e1.origin() - e2.origin();
        const Vec2 s = A.colPivHouseholderQr().solve(b);

        const Vec3 int1 = e1.origin() + e1.direction() * s(0);
        const Vec3 int2 = e2.origin() + e2.direction() * s(1);
        assert(mdv::condition::are_equal(int1, int2));
        intersection_point = int1;
        sols               = s;
    }

    Vec2 sols;
    Vec3 intersection_point;
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_HELPERS_HPP
