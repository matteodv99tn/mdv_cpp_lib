#ifndef MDV_FLAT_PARAMETERISATION_HPP
#define MDV_FLAT_PARAMETERISATION_HPP

#include <gsl/pointers>

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"

namespace mdv::mesh {

class FlatParameterisation {
public:
    FlatParameterisation(Mesh& mesh);

    FlatParameterisation(const FlatParameterisation& other) = delete;
    FlatParameterisation(FlatParameterisation&&) noexcept;
    FlatParameterisation& operator=(const FlatParameterisation&) = delete;
    FlatParameterisation& operator=(FlatParameterisation&&) noexcept;

    ~FlatParameterisation();

    MDV_NODISCARD bool is_one_to_one_mapping() const;

    MDV_NODISCARD Eigen::Vector2d project(const Point& pt) const;

    MDV_NODISCARD Point retrieve(const Eigen::Vector2d& uv) const;

    MDV_NODISCARD bool is_inside_mesh(const Eigen::Vector2d& uv) const;

    MDV_NODISCARD Eigen::Vector2d min_uv() const;

    MDV_NODISCARD Eigen::Vector2d max_uv() const;

private:
    class FlatParameterisationImpl;

    Mesh*                                 _mesh;
    gsl::owner<FlatParameterisationImpl*> _impl = nullptr;
};

}  // namespace mdv::mesh


#endif  // MDV_FLAT_PARAMETERISATION_HPP
