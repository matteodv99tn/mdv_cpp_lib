#ifndef MDV_MESH_CLASS_HPP
#define MDV_MESH_CLASS_HPP

#include <Eigen/Geometry>
#include <filesystem>
#include <gsl/pointers>
#include <spdlog/fwd.h>
#include <string_view>

#include <boost/range/iterator_range.hpp>
#include <boost/range/iterator_range_core.hpp>

#include "mdv/macros.hpp"
#include "mdv/mesh/face.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh_iterator.hpp"
#include "mdv/mesh/point.hpp"
#include "mdv/mesh/vertex.hpp"

namespace mdv::mesh {

class Mesh {
public:
    // Forward declarations
    using Vertex = ::mdv::mesh::Vertex;
    using Face   = ::mdv::mesh::Face;
    using Point  = ::mdv::mesh::Point;

    using MeshData  = ::mdv::mesh::internal::MeshData;
    using EigenData = ::mdv::mesh::internal::EigenData;
    using CgalImpl  = ::mdv::mesh::internal::CgalImpl;

    friend class ::mdv::mesh::Vertex;

    using Geodesic = ::mdv::mesh::Geodesic;

    // enum GeodesicBuilderPolicy : std::uint8_t {
    //     InternalState,
    //     ExecutionLocal
    // };

    // Factory functions
    static Mesh from_file(const std::filesystem::path& file_path);

    Mesh(const Mesh& other)            = delete;
    Mesh(const Mesh&& other)           = delete;
    Mesh operator=(const Mesh& other)  = delete;
    Mesh operator=(const Mesh&& other) = delete;

    void transform(const Eigen::Affine3d& transformation);

    /**
     * @brief Builds a geodesic path between two points.
     *
     * @param from The starting point of the geodesic.
     * @param to The ending point of the geodesic.
     *
     * @return Geodesic The constructed geodesic path.
     */
    MDV_NODISCARD Geodesic build_geodesic(const Point& from, const Point& to) const;

    MDV_NODISCARD std::vector<Eigen::Vector3d> compute_vertex_normals() const noexcept;


    // Getters
    MDV_NODISCARD std::size_t num_vertices() const;
    MDV_NODISCARD std::size_t num_faces() const;

    MDV_NODISCARD std::string_view
                  name() const {
                      return data().name;
    };

    // clang-format off
    MDV_NODISCARD Face            face(const Index& id) const   { return {_data, id}; }
    MDV_NODISCARD Vertex          vertex(const Index& id) const { return {_data, id}; }
    MDV_NODISCARD spdlog::logger& logger() const                { assert(_data.logger != nullptr); return *_data.logger.get(); };

    // clang-format on

    //  ___ _                 _
    // |_ _| |_ ___ _ __ __ _| |_ ___  _ __ ___
    //  | || __/ _ \ '__/ _` | __/ _ \| '__/ __|
    //  | || ||  __/ | | (_| | || (_) | |  \__ \
    // |___|\__\___|_|  \__,_|\__\___/|_|  |___/
    //

    // using FaceIterator = MeshIterator<Face>;
    using FaceIterator   = Face::Iterator;
    using VertexIterator = MeshIterator<Vertex, Index>;

    MDV_NODISCARD FaceIterator faces_begin() const noexcept;
    MDV_NODISCARD FaceIterator faces_end() const noexcept;
    MDV_NODISCARD boost::iterator_range<FaceIterator> faces() const noexcept;

    MDV_NODISCARD VertexIterator vertices_begin() const noexcept;
    MDV_NODISCARD VertexIterator vertices_end() const noexcept;
    MDV_NODISCARD boost::iterator_range<VertexIterator> vertices() const noexcept;

    // clang-format off
    MDV_NODISCARD MeshData&        data() noexcept             { return _data; }
    MDV_NODISCARD const MeshData&  data() const noexcept       { return _data; }
    MDV_NODISCARD CgalImpl&        cgal()                      { assert(_data.impl); return *_data.impl; }
    MDV_NODISCARD const CgalImpl&  cgal() const                { assert(_data.impl); return *_data.impl; }
    MDV_NODISCARD EigenData&       eigen_data() noexcept       { return _data.eigen_data; }
    MDV_NODISCARD const EigenData& eigen_data() const noexcept { return _data.eigen_data; }

    // clang-format on


private:
    Mesh(gsl::owner<CgalImpl*> data, const std::string& name);

    // Members
    MeshData _data;
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_CLASS_HPP
