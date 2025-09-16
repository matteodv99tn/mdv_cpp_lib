#include "mdv/dmp/learnable_function.hpp"

#include "mdv/dmp/dmp.hpp"
#include "mdv/riemann_geometry/s3.hpp"
#include "mdv/riemann_geometry/scalar.hpp"

using mdv::dmp::LearnableFunction;
using mdv::riemann::S3;
using mdv::riemann::Scalar;

// S3 testing
static_assert(std::is_same_v<
              LearnableFunction<mdv::embedding_dimension<S3::TangentVector>>::Output,
              Eigen::Vector4d>);
static_assert(std::is_same_v<
              LearnableFunction<mdv::embedding_dimension<S3::TangentVector>>::Output,
              Eigen::Vector4d>);

// Scalar testing
static_assert(std::is_same_v<
              LearnableFunction<
                      mdv::embedding_dimension<Scalar::TangentVector>>::Output,
              double>);
