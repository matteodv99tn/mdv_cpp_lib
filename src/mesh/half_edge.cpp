#include "mdv/mesh/half_edge.hpp"

#include <fmt/format.h>

#include "mdv/utils/conditions.hpp"

using mdv::mesh::HalfEdge;

Eigen::Vector3d
HalfEdge::inbound_direction() const {
    using Vec3   = Eigen::Vector3d;
    using Mat3   = Eigen::Matrix3d;
    const Vec3 e = normalised_direction();          // current edge
    const Vec3 v = next()->normalised_direction();  // vector w.r.t. which will be
                                                    // rendered orthoghonal to e

    const Mat3 proj = (Mat3::Identity() - e * e.transpose());
    const Vec3 res  = (proj * v).normalized();

    assert(mdv::condition::are_orthogonal(e, res));
    assert(mdv::condition::are_orthogonal(face().normal(), res));

    return res;
}

Eigen::Quaterniond
HalfEdge::aligning_rotation() const {
    using Vec3 = Eigen::Vector3d;
    using Quat = Eigen::Quaterniond;

    const Vec3 e       = normalised_direction();
    const Vec3 x_this  = inbound_direction();
    const Vec3 x_other = twin()->inbound_direction();

    assert(mdv::condition::are_orthogonal(x_this, e));
    assert(mdv::condition::are_orthogonal(x_other, e));

    const Quat res = Quat::FromTwoVectors(-x_this, x_other);
    assert(mdv::condition::are_equal(res * (-x_this), x_other));
    assert(mdv::condition::are_equal(res * e, e));
    return res;
}

std::string
HalfEdge::describe() const {
    return fmt::format(
            "HalfEdge on face #{} with origin vertex #{}",
            (_face != nullptr) ? _face->id() : -1,
            _origin->id()
    );
}
