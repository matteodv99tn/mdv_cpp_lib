#ifndef MDV_MESH_TANGENT_VECTOR_HPP
#define MDV_MESH_TANGENT_VECTOR_HPP

#include <mdv/mesh/mesh.hpp>
#include <mdv/mesh/uv_map.hpp>

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/utils/logging.hpp"

namespace mdv::mesh {

class TangentVector {
public:
    using Point   = Mesh::Point;
    using UvCoord = Mesh::Point::UvCoord;

    TangentVector(const Point& app_point, const UvCoord& uv) :
            _application_point(app_point), _uv(uv) {};
    TangentVector(const Point& app_point, const Vec3d& v);

    /**
     * @brief Constructs a tangent vector given its application point and the
     * "tip" position of the vector.
     */
    static TangentVector from_tip_position(
            const Mesh::Point& application_point, const Point3d& tip
    );

    static TangentVector unit_random(const Mesh::Point& application_point);

    MDV_NODISCARD Point3d tip() const noexcept;

    MDV_NODISCARD Vec3d cartesian_vector() const noexcept;

    /**
     * @brief Trims a vector and return the "remainder" projected.
     *
     * Generally, a tangent vector could extend outside the boarder of the face.
     * This function virtually "cuts" the vector at the edge, and return the remainder
     * of the vector projected on the contiguous face based on conformal mappings.
     *
     * Note that if the vector was able to fit within the triangular face, it is left
     * unchanged.
     *
     */
    std::optional<TangentVector> trim();

    /**
     * @brief Scales the tangent vector length by the provided factor, i.e.
     *      v_new = factor * v
     *
     */
    void scale(const double& factor);

    /**
     * @brief Normalise the vector to make sure it has unitary norm (in cartesian
     * space).
     *
     */
    void normalise();

    /**
     * @brief Returns a (new) normalised tangent vector.
     *
     */
    TangentVector normalised() &;
    TangentVector normalised() &&;

    // clang-format off
    MDV_NODISCARD const UvCoord& uv() const                { return _uv; }
    MDV_NODISCARD const UvMap&   uv_map() const            { return _application_point.uv_map(); }
    MDV_NODISCARD const Point&   application_point() const { return _application_point; }

    MDV_NODISCARD SpdLoggerPtr&  logger() const            { return application_point().logger(); }

    // clang-format on

private:
    Point   _application_point;
    UvCoord _uv;

    MDV_NODISCARD
    const UvMap::Transform&
    jac() const {
        return _application_point.uv_map().forward_map_jacobian();
    }
};


}  // namespace mdv::mesh


#endif  // MDV_MESH_TANGENT_VECTOR_HPP
