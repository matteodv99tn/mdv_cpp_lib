#include "mdv/riemann_geometry/s3.hpp"

#include "mdv/riemann_geometry/hypershere.hpp"

using S3     = mdv::riemann::S3;
using S3Impl = mdv::riemann::S<3>;

S3::TangentVector
S3::logarithmic_map(const Point& q1, const Point& q2) {
    return S3Impl::logarithmic_map(q1.coeffs(), q2.coeffs());
}

S3::Point
S3::exponential_map(const Point& q, const TangentVector& v) {
    return Point(S3Impl::exponential_map(q.coeffs(), v));
}

S3::TangentVector
S3::parallel_transport(const Point& q, const Point& p, const TangentVector& v) {
    return S3Impl::parallel_transport(q.coeffs(), p.coeffs(), v);
}

S3::TangentVector
S3::covariant_derivative(const Point& q, const TangentVector& v) {
    return S3Impl::covariant_derivative(q.coeffs(), v);
}

S3::Point
S3::default_point() {
    return Point::Identity();
}

S3::TangentVector
S3::default_tangent_vector() {
    return TangentVector::Zero();
}
