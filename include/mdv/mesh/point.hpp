#ifndef MDV_MESH_POINT_HPP
#define MDV_MESH_POINT_HPP

#include "mdv/mesh/face.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh_data.hpp"
#include "mdv/mesh/uv_map.hpp"

namespace mdv::mesh {

class Point {
public:
    using Face     = ::mdv::mesh::Face;
    using UvCoord  = UvMap::Domain;
    using MeshData = ::mdv::mesh::internal::MeshData;

    Point() : _face(), _uv(0.0, 0.0), _uv_map() {};

    Point(const Face& face, const UvCoord& uv) :
            _face(face), _uv(uv), _uv_map(_face.compute_uv_map()) {}

    /**
     * @brief Retrieves the closes point on the mesh to the given point described in
     * 3D space.
     *
     */
    static Point from_cartesian(const Mesh& m, const CartesianPoint& pt);

    /**
     * @brief Initialise the location of a point in 3d space given the face it shall
     * be parameterised in
     *
     */
    static Point from_face_and_position(const Face& m, const CartesianPoint& pt);

    /**
     * @brief Retrieves the closes point on the mesh to the given point described in
     * 3D space.
     *
     */
    static Point from_barycentric(const Mesh& m, const CartesianPoint& barycentric);

    /**
     * @brief Defines an "undefined" point, i.e. a point with no meaning.
     *
     * Useful to initialise a point into a known state that represent an invalid
     * location.
     *
     */
    static Point undefined(const Mesh& m) noexcept;

    static Point random(const Mesh& m) noexcept;

    MDV_NODISCARD bool is_undefined() const noexcept;

    /**
     * @brief Return the barycentric coordinate of the point.
     */
    MDV_NODISCARD Eigen::Vector3d barycentric() const noexcept;

    /**
     * @brief Retrieves the cartesian position of the point.
     *
     */
    MDV_NODISCARD CartesianPoint position() const noexcept;

    void  constrain_inside_triangle() &;
    Point constrain_inside_triangle() &&;

    MDV_NODISCARD
    std::string describe() const;

    // clang-format off
    MDV_NODISCARD double          u() const noexcept         { return _uv(0); }
    MDV_NODISCARD double          v() const noexcept         { return _uv(1); }
    MDV_NODISCARD const Face&     face() const noexcept      { return _face; }
    MDV_NODISCARD Face::Index     face_id() const noexcept   { return _face.id(); }
    MDV_NODISCARD const MeshData& data() const noexcept      { return _face.data(); }
    MDV_NODISCARD spdlog::logger& logger() const noexcept    { return _face.logger(); }
    MDV_NODISCARD const UvCoord&  uv() const noexcept        { return _uv; }
    MDV_NODISCARD const UvMap&    uv_map() const noexcept    { return _uv_map; }

    MDV_NODISCARD bool operator==(const Point& other) const noexcept;
    MDV_NODISCARD bool operator!=(const Point& other) const noexcept;
    // clang-format on
private:
    Point(const Face& face, const CartesianPoint& pt);

    Face    _face;
    UvCoord _uv;
    UvMap   _uv_map;  // Maybe consider caching this variable
};


}  // namespace mdv::mesh


#endif  // MDV_MESH_POINT_HPP
