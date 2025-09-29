#ifndef MDV_MESH_MANIFOLD_HPP
#define MDV_MESH_MANIFOLD_HPP

#include <Eigen/Dense>

#include "mdv/mesh/mesh.hpp"

namespace mdv::riemann {

class MeshManifold {
public:
    using Point         = mdv::mesh::Point;
    using TangentVector = Eigen::Vector3d;

    TangentVector logarithmic_map(const Point& q1, const Point& q2) const;

    Point exponential_map(const Point& q, const TangentVector& v) const;

    TangentVector parallel_transport(
            const Point& q, const Point& p, const TangentVector& v
    ) const;

    TangentVector covariant_derivative(const Point& q, const TangentVector& v) const;

    Point default_point() const;

    TangentVector default_tangent_vector() const;
};

struct MeshEmbedder {
    using M      = mdv::riemann::MeshManifold;
    using Input  = M::TangentVector;
    using Output = Eigen::Vector2d;

    using Vec3 = Eigen::Vector3d;
    using Mat3 = Eigen::Matrix3d;

    MeshEmbedder(const M* manifold) : _m(manifold) {};

    void
    setup(const M::Point& y0, const M::Point& g) {
        using mdv::condition::are_orthogonal, mdv::condition::is_zero;
        _y0 = y0;
        _g  = g;

        const Vec3 vx = -_m->logarithmic_map(g, y0).normalized();
        const Vec3 vz = g.face().normal();
        const Vec3 vy = vz.cross(vx);

        _base.col(0) = vx;
        _base.col(1) = vy;
        _base.col(2) = vz;

        assert(are_orthogonal(vx, vy));
        assert(are_orthogonal(vx, vz));
        assert(are_orthogonal(vy, vz));
        assert(is_zero(_base.determinant() - 1.0));

        _initialised = true;
    }

    template <typename StateType, typename GoalType>
    Output
    embed(const Input& in, const StateType& x, const GoalType& g) const {
        if (!_initialised) throw std::runtime_error("MeshEmbedder not initialised!");

        const Vec3 v_in_g   = _m->parallel_transport(x.y(), g.y(), in);
        const Vec3 v_coords = _base.inverse() * v_in_g;
        if (!mdv::condition::is_zero(v_coords(2))) std::terminate();
        return {v_coords(0), v_coords(1)};
    }

    template <typename StateType, typename GoalType>
    Input
    decode(const Output& out, const StateType& x, const GoalType& g) const {
        using mdv::condition::are_orthogonal;

        if (!_initialised) throw std::runtime_error("MeshEmbedder not initialised!");

        const Vec3 v_coords{out(0), out(1), 0.0};
        const Vec3 v_in_g = _base * v_coords;

        assert(g.y() == _g);
        assert(are_orthogonal(v_in_g, g.y().face().normal()));
        assert(are_orthogonal(v_in_g, _g.face().normal()));

        const Vec3 res = _m->parallel_transport(g.y(), x.y(), v_in_g);
        if (!mdv::condition::are_orthogonal(res, x.y().face().normal())) {
            fmt::println("Scalar prod: {}", res.dot(_y0.face().normal()));
        }

        assert(are_orthogonal(res, x.y().face().normal()));
        // assert(are_orthogonal(res, _y0.face().normal()));
        return res;
    }

private:
    const M* _m;
    M::Point _y0;
    M::Point _g;
    Mat3     _base;
    bool     _initialised = false;
};

}  // namespace mdv::riemann

#endif  // MDV_MESH_MANIFOLD_HPP
