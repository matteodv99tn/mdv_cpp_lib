#include "mdv/dmp/concepts.hpp"
#include "mdv/riemann_geometry/concepts.hpp"
#include "mdv/riemann_geometry/euclidean.hpp"
#include "mdv/riemann_geometry/fwd.hpp"
#include "mdv/riemann_geometry/s3.hpp"
#include "mdv/riemann_geometry/scalar.hpp"
#include "mdv/riemann_geometry/se3.hpp"

using namespace mdv::concepts;
using namespace mdv::riemann;

// static_assert(
//         mdv::riemann::space_dimension_v<mdv::riemann::Scalar::TangentVector> == 1
// );
// static_assert(mdv::riemann::space_dimension_v<mdv::riemann::S3::TangentVector> == 4);
// static_assert(mdv::riemann::space_dimension_v<mdv::riemann::SE3::TangentVector> ==
// 7);

static_assert(manifold<Scalar>);
static_assert(manifold<Rn<3>>);
static_assert(manifold<S3>);
static_assert(manifold<SE3>);

static_assert(euclidean_space<Scalar>);
static_assert(euclidean_space<Rn<3>>);
static_assert(!euclidean_space<S3>);
static_assert(!euclidean_space<SE3>);

static_assert(trivially_embeddable_manifold<Scalar>);
static_assert(trivially_embeddable_manifold<Rn<3>>);
static_assert(trivially_embeddable_manifold<S3>);
static_assert(trivially_embeddable_manifold<SE3>);

static_assert(
        space_dimension_v<TrivialTypeEmbedding<Scalar::TangentVector>::Output> == 1
);
static_assert(
        space_dimension_v<TrivialTypeEmbedding<Rn<3>::TangentVector>::Output> == 3
);
static_assert(space_dimension_v<TrivialTypeEmbedding<S3::TangentVector>::Output> == 4);
static_assert(space_dimension_v<TrivialTypeEmbedding<SE3::TangentVector>::Output> == 7);
