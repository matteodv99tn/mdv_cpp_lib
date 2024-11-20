#ifndef MDV_MESH_CLASS_HPP
#define MDV_MESH_CLASS_HPP

#include <array>
#include <Eigen/Geometry>
#include <filesystem>
#include <gsl/pointers>
#include <utility>

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
    class Point;
    class CgalMesh;

    using Geodesic = ::mdv::mesh::Geodesic;

    enum GeodesicBuilderPolicy : std::uint8_t {
        InternalState,
        ExecutionLocal
    };

    class IndexBasedElement {
    public:
        using Index = long;

        IndexBasedElement(const Mesh& m, const Index& id) noexcept :
                _mesh(&m), _id(id) {};

        // clang-format off
        MDV_NODISCARD Index id() const noexcept { return _id; }
        MDV_NODISCARD const Mesh& mesh() const noexcept { return *_mesh; }

        // clang-format on

    protected:
        gsl::not_null<const Mesh*> _mesh;
        Index                      _id;
    };

    // __     __        _
    // \ \   / /__ _ __| |_ _____  __
    //  \ \ / / _ \ '__| __/ _ \ \/ /
    //   \ V /  __/ |  | ||  __/>  <
    //    \_/ \___|_|   \__\___/_/\_\
    //
    class Vertex : public IndexBasedElement {
    public:
        using Index    = long;
        using Iterator = MeshIterator<Vertex, Index>;

        Vertex(const Mesh& m, const Index& id) noexcept : IndexBasedElement(m, id) {};

        MDV_NODISCARD Eigen::Vector3d normal() const noexcept;

        // clang-format off

        bool operator==(const Vertex& other) const noexcept { return (_mesh == other._mesh) && (_id == other._id); }

        MDV_NODISCARD const Point3d&
        position() const noexcept {return _mesh->_v_mat[_id]; }

        // clang-format on

    private:
        friend class MeshIterator<Vertex, Index>;
    };

    //  _____
    // |  ___|_ _  ___ ___
    // | |_ / _` |/ __/ _ \
    // |  _| (_| | (_|  __/
    // |_|  \__,_|\___\___|
    //
    class Face : public IndexBasedElement {
    public:
        using Index         = long;
        using IndexTriplet  = std::array<Index, 3>;
        using Iterator      = MeshIterator<Face, Index>;
        using VertexTriplet = std::array<Vertex, 3>;
        Face(const Mesh& m, const Index& id) noexcept : IndexBasedElement(m, id) {};

        static Face random(const Mesh& m) noexcept;

        /**
         * @brief Normal vector to the triangle
         *
         */
        MDV_NODISCARD Vec3d normal() const;

        /**
         * @brief Yields the vertex of the face using face indexing, zero-based.
         *
         * @param[in] i  index of the vertex. Checked to be in the range [0, 2].
         */
        MDV_NODISCARD Vertex vertex(long i) const;

        /**
         * @brief Yields the set of vertices that make up the face.
         */
        MDV_NODISCARD VertexTriplet vertices() const;

        // clang-format off
        /**
         * @brief Returns the (ordered) triplet of indices of the vertex indices.
         *
         */
        MDV_NODISCARD const IndexTriplet& 
        vertices_ids() const { return _mesh->_f_mat[_id]; }

        bool operator==(const Face& other) const noexcept { return (_mesh == other._mesh) && (_id == other._id); }

        // clang-format on

    private:
        friend class MeshIterator<Face, Index>;
        friend class Point;

        void
        set_id(const Index& id) noexcept {
            _id = id;
        }

        /**
         * @brief Return the position of the first vertex within the face.
         *
         */
        MDV_NODISCARD const Point3d&
        v1() const noexcept {
            return _mesh->_v_mat[_mesh->_f_mat[_id][0]];
        };

        /**
         * @brief Return the position of the second vertex within the face.
         *
         */
        MDV_NODISCARD const Point3d&
        v2() const noexcept {
            return _mesh->_v_mat[_mesh->_f_mat[_id][1]];
        };

        /**
         * @brief Return the position of the third vertex within the face.
         *
         */
        MDV_NODISCARD const Point3d&
        v3() const noexcept {
            return _mesh->_v_mat[_mesh->_f_mat[_id][2]];
        };
    };

    //  ____       _       _
    // |  _ \ ___ (_)_ __ | |_
    // | |_) / _ \| | '_ \| __|
    // |  __/ (_) | | | | | |_
    // |_|   \___/|_|_| |_|\__|
    //
    /**
     * @brief Class that describes a point lying on the surface of the mesh.
     *
     * Internally the point is stored as a pair of UV coordinates on a given face.
     */
    class Point {
    public:
        Point(Face face, Eigen::Vector2d uv) :
                _face(std::move(face)), _uv(std::move(uv)) {}

        /**
         * @brief Retrieves the closes point on the mesh to the given point described in
         * 3D space.
         *
         */
        static Point from_cartesian(const Mesh& m, const Point3d& pt);

        /**
         * @brief Retrieves the closes point on the mesh to the given point described in
         * 3D space.
         *
         */
        static Point from_barycentric(const Mesh& m, const Point3d& barycentric);

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
        MDV_NODISCARD Point3d position() const noexcept;

        // clang-format off
        MDV_NODISCARD double u() const noexcept { return _uv(0); }
        MDV_NODISCARD double v() const noexcept { return _uv(1); }
        MDV_NODISCARD const Face& face() const noexcept { return _face; }
        MDV_NODISCARD Face::Index face_id() const noexcept { return _face.id(); }
        MDV_NODISCARD const Mesh& mesh() const noexcept { return _face.mesh(); }
        MDV_NODISCARD const Eigen::Vector2d& uv() const noexcept { return _uv; }

        // clang-format on
    private:
        Face            _face;
        Eigen::Vector2d _uv;
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

    /**
     * @brief Builds a geodesic path between two points.
     *
     * Available policies are:
     *  - InternalState: uses an internal state that remembers the last "to" location;
     *    to be preffered when it is known that the end-point for the geodesic is not
     *    changing between different function calls;
     *  - ExecutionLocal: creates a local ShortestPath object; slowest method.
     *
     * @param from The starting point of the geodesic.
     * @param to The ending point of the geodesic.
     * @param policy The policy to use for building the geodesic. Defaults to
     * InternalState.
     * @return Geodesic The constructed geodesic path.
     */
    MDV_NODISCARD Geodesic build_geodesic(
            const Point&          from,
            const Point&          to,
            GeodesicBuilderPolicy policy = InternalState
    ) const;

    MDV_NODISCARD std::vector<Eigen::Vector3d> compute_vertex_normals() const noexcept;


    // Getters

    /**
     * @brief Provides the file name, without extension, of the source file of
     * the mesh.
     *
     */
    [[nodiscard]] std::string file_name() const;

    MDV_NODISCARD std::size_t num_vertices() const;
    MDV_NODISCARD std::size_t num_faces() const;

    // clang-format off
    MDV_NODISCARD Face face(const Face::Index& id) const { return {*this, id}; }
    MDV_NODISCARD Vertex vertex(const Vertex::Index& id) const { return {*this, id}; }

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
    SpdLoggerPtr          _logger;
    gsl::owner<CgalMesh*> _cgal_data{nullptr};
    std::filesystem::path _file_path;
    friend class CgalMesh;

    std::vector<Point3d>            _v_mat;
    std::vector<Face::IndexTriplet> _f_mat;

    std::vector<Face::IndexTriplet> _neighbouring_faces;
};

// #include <mdv/mesh/face_iterator.hpp>

}  // namespace mdv::mesh


#endif  // MDV_MESH_CLASS_HPP
