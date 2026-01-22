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

    MDV_NODISCARD Eigen::Vector2d project(const Point& pt) const;

private:
    class FlatParameterisationImpl;

    Mesh*                                 _mesh;
    gsl::owner<FlatParameterisationImpl*> _impl = nullptr;
};

}  // namespace mdv::mesh


#endif  // MDV_FLAT_PARAMETERISATION_HPP
