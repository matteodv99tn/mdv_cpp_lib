#ifndef MDV_MESH_CLASS_HPP
#define MDV_MESH_CLASS_HPP

#include <array>
#include <Eigen/Geometry>
#include <filesystem>
#include <gsl/pointers>

#include <boost/range/iterator_range.hpp>
#include <boost/range/iterator_range_core.hpp>
#include <mdv/eigen_defines.hpp>
#include <mdv/macros.hpp>
#include <mdv/mesh/fwd.hpp>
#include <mdv/mesh/mesh_iterator.hpp>
#include <mdv/utils/logging.hpp>

namespace mdv::mesh {

class Mesh {
public:
    // Forward declarations
    class Vertex;
    class Face;

    // __     __        _
    // \ \   / /__ _ __| |_ _____  __
    //  \ \ / / _ \ '__| __/ _ \ \/ /
    //   \ V /  __/ |  | ||  __/>  <
    //    \_/ \___|_|   \__\___/_/\_\
    //
    class Vertex {
    public:
        using Index    = long;
        using Iterator = MeshIterator<Vertex, Index>;

        Vertex(const Mesh* m, const Index& id) noexcept : _mesh(m), _id(id) {};

        MDV_NODISCARD Point3d_t position() const noexcept;

        // clang-format off

        MDV_NODISCARD long id() const { return _id; }
        bool operator==(const Vertex& other) const noexcept { return (_mesh == other._mesh) && (_id == other._id); }

        // clang-format on

    private:
        const Mesh* _mesh;
        Index       _id;
        friend class MeshIterator<Vertex, Index>;
    };

    //  _____
    // |  ___|_ _  ___ ___
    // | |_ / _` |/ __/ _ \
    // |  _| (_| | (_|  __/
    // |_|  \__,_|\___\___|
    //
    class Face {
    public:
        using Index         = long;
        using IndexTriplet  = std::array<Index, 3>;
        using Iterator      = MeshIterator<Face, Index>;
        using VertexTriplet = std::array<Vertex, 3>;
        Face(const Mesh* m, const Index& id) noexcept : _mesh(m), _id(id) {};

        MDV_NODISCARD IndexTriplet search_neighbour_faces_ids() const noexcept;

        /**
         * @brief Normal vector to the triangle
         *
         */
        MDV_NODISCARD Vec3d normal() const;

        /**
         * @brief Yields the vertex of the face using face indexing, zero-based.
         *
         * @param[in] i  index of the vertex. Checkd to be in the range [0, 2].
         */
        MDV_NODISCARD Vertex vertex(long i) const;

        /**
         * @brief Yields the set of vertices that make up the face.
         */
        MDV_NODISCARD VertexTriplet vertices() const;

        // clang-format off
        MDV_NODISCARD const IndexTriplet& vertices_ids() const { return _mesh->_f_mat[_id]; }

        MDV_NODISCARD long id() const { return _id; }
        bool operator==(const Face& other) const noexcept { return (_mesh == other._mesh) && (_id == other._id); }

        // clang-format on

    private:
        const Mesh* _mesh;
        Index       _id;
        friend class MeshIterator<Face, Index>;
    };

    //  __  __           _
    // |  \/  | ___  ___| |__
    // | |\/| |/ _ \/ __| '_ \
    // | |  | |  __/\__ \ | | |
    // |_|  |_|\___||___/_| |_|
    //

    /**
     * @brief Initialises a mesh object from file
     *
     * @param[input] file_path path to the source file of the mesh
     */
    Mesh(const std::filesystem::path& file_path);

    Mesh(const Mesh& other)            = delete;
    Mesh(const Mesh&& other)           = delete;
    Mesh operator=(const Mesh& other)  = delete;
    Mesh operator=(const Mesh&& other) = delete;

    ~Mesh();

    void transform(const Eigen::Affine3d& transformation);


    // Getters

    /**
     * @brief Provides the file name, without extension, of the source file of the mesh.
     *
     */
    [[nodiscard]] std::string file_name() const;

    MDV_NODISCARD std::size_t num_vertices() const;
    MDV_NODISCARD std::size_t num_faces() const;

    // clang-format off
    MDV_NODISCARD Face face(const Face::Index& id) const { return {this, id}; }
    MDV_NODISCARD Vertex vertex(const Vertex::Index& id) const { return {this, id}; }

    // clang-format on

    //  ___ _                 _
    // |_ _| |_ ___ _ __ __ _| |_ ___  _ __ ___
    //  | || __/ _ \ '__/ _` | __/ _ \| '__/ __|
    //  | || ||  __/ | | (_| | || (_) | |  \__ \
    // |___|\__\___|_|  \__,_|\__\___/|_|  |___/
    //

    // using FaceIterator = MeshIterator<Face>;
    using FaceIterator   = Face::Iterator;
    using VertexIterator = Vertex::Iterator;

    MDV_NODISCARD FaceIterator faces_begin() const noexcept;
    MDV_NODISCARD FaceIterator faces_end() const noexcept;
    MDV_NODISCARD boost::iterator_range<FaceIterator> faces() const noexcept;

    MDV_NODISCARD VertexIterator vertices_begin() const noexcept;
    MDV_NODISCARD VertexIterator vertices_end() const noexcept;
    MDV_NODISCARD boost::iterator_range<VertexIterator> vertices() const noexcept;


private:
    /**
     * @brief Syncs internal vertex data storage with the one of CGAL.
     */
    void sync_vertex_data();

    /**
     * @brief Syncs internal face data storage with the one of CGAL.
     */
    void sync_face_data();


    // Members
    class CgalMesh;
    LoggerPtr_t           _logger;
    gsl::owner<CgalMesh*> _cgal_data{nullptr};
    std::filesystem::path _file_path;

    Eigen::Matrix<double, 3, Eigen::Dynamic> _v_mat;
    std::vector<Face::IndexTriplet>          _f_mat;

    std::vector<Face::IndexTriplet> _neighbouring_faces;
};

// #include <mdv/mesh/face_iterator.hpp>

}  // namespace mdv::mesh


#endif  // MDV_MESH_CLASS_HPP
