#ifndef MDV_FLAT_PARAMETERISATION_HPP
#define MDV_FLAT_PARAMETERISATION_HPP

#include <gsl/pointers>

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"

namespace mdv::mesh {

/**
 * @brief 2D parameterization of a surface mesh.
 *
 * Provides UV mapping for projecting 3D surface points to a planar domain and
 * lifting them back, which is useful for visualization, learning in 2D, and
 * building surface-conditioned policies.
 */
class FlatParameterisation {
public:
    /**
     * @brief Constructs a parameterization for a mesh.
     *
     * @param mesh Mesh to parameterize.
     */
    FlatParameterisation(Mesh& mesh);

    /**
     * @brief Non-copyable.
     */
    FlatParameterisation(const FlatParameterisation& other) = delete;

    /**
     * @brief Move constructor.
     *
     * @param other Parameterisation to move from.
     */
    FlatParameterisation(FlatParameterisation&&) noexcept;

    /**
     * @brief Non-copyable.
     */
    FlatParameterisation& operator=(const FlatParameterisation&) = delete;

    /**
     * @brief Move assignment.
     *
     * @param other Parameterisation to move from.
     * @return Reference to this parameterisation.
     */
    FlatParameterisation& operator=(FlatParameterisation&&) noexcept;

    /**
     * @brief Destructor.
     */
    ~FlatParameterisation();

    /**
     * @brief True if the parameterization is one-to-one.
     *
     * @return True if one-to-one mapping.
     */
    MDV_NODISCARD bool is_one_to_one_mapping() const;

    /**
     * @brief Projects a surface point to UV coordinates.
     *
     * @param pt Point on the mesh.
     * @return UV coordinates.
     */
    MDV_NODISCARD Eigen::Vector2d project(const Point& pt) const;

    /**
     * @brief Lifts a UV point back to the mesh surface.
     *
     * @param uv UV coordinates.
     * @return Point on the mesh.
     */
    MDV_NODISCARD Point retrieve(const Eigen::Vector2d& uv) const;

    /**
     * @brief Returns true if the UV point lies inside the parameterized mesh.
     *
     * @param uv UV coordinates.
     * @return True if inside the mesh domain.
     */
    MDV_NODISCARD bool is_inside_mesh(const Eigen::Vector2d& uv) const;

    /**
     * @brief Minimum UV coordinate in the parameterization.
     *
     * @return Minimum UV values.
     */
    MDV_NODISCARD Eigen::Vector2d min_uv() const;

    /**
     * @brief Maximum UV coordinate in the parameterization.
     *
     * @return Maximum UV values.
     */
    MDV_NODISCARD Eigen::Vector2d max_uv() const;

private:
    class FlatParameterisationImpl;

    Mesh*                                 _mesh;
    gsl::owner<FlatParameterisationImpl*> _impl = nullptr;
};

}  // namespace mdv::mesh


#endif  // MDV_FLAT_PARAMETERISATION_HPP
