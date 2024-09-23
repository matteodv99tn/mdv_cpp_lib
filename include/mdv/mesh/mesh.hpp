#ifndef MDV_MESH_CLASS_HPP
#define MDV_MESH_CLASS_HPP

#include <Eigen/Geometry>
#include <filesystem>
#include <gsl/pointers>

#include <boost/range/iterator_range.hpp>
#include <boost/range/iterator_range_core.hpp>
#include <mdv/macros.hpp>
#include <mdv/mesh/face.hpp>
#include <mdv/mesh/fwd.hpp>
#include <mdv/mesh/vertex.hpp>
#include <mdv/utils/logging.hpp>

namespace mdv::mesh {

class Mesh {
public:
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

    MDV_NODISCARD VertexIndex_t num_vertices() const;
    MDV_NODISCARD FaceIndex_t   num_faces() const;
    MDV_NODISCARD Vertex        vertex(const VertexIndex_t& id);
    MDV_NODISCARD Vertex        vertex(const VertexIndex_t& id) const;
    MDV_NODISCARD Face          face(const FaceIndex_t& id);
    MDV_NODISCARD Face          face(const FaceIndex_t& id) const;


    //  ___ _                 _
    // |_ _| |_ ___ _ __ __ _| |_ ___  _ __ ___
    //  | || __/ _ \ '__/ _` | __/ _ \| '__/ __|
    //  | || ||  __/ | | (_| | || (_) | |  \__ \
    // |___|\__\___|_|  \__,_|\__\___/|_|  |___/
    //
    MDV_NODISCARD VertexIterator      vertices_begin() noexcept;
    MDV_NODISCARD VertexIterator      vertices_end() noexcept;
    MDV_NODISCARD ConstVertexIterator cvertices_begin() const noexcept;
    MDV_NODISCARD ConstVertexIterator cvertices_end() const noexcept;
    MDV_NODISCARD ConstVertexIterator vertices_begin() const noexcept;
    MDV_NODISCARD ConstVertexIterator vertices_end() const noexcept;
    MDV_NODISCARD boost::iterator_range<VertexIterator> vertices() noexcept;
    MDV_NODISCARD boost::iterator_range<ConstVertexIterator> vertices() const noexcept;
    MDV_NODISCARD boost::iterator_range<ConstVertexIterator> cvertices() const noexcept;

    MDV_NODISCARD FaceIterator      faces_begin() noexcept;
    MDV_NODISCARD FaceIterator      faces_end() noexcept;
    MDV_NODISCARD ConstFaceIterator cfaces_begin() const noexcept;
    MDV_NODISCARD ConstFaceIterator cfaces_end() const noexcept;
    MDV_NODISCARD ConstFaceIterator faces_begin() const noexcept;
    MDV_NODISCARD ConstFaceIterator faces_end() const noexcept;
    MDV_NODISCARD boost::iterator_range<FaceIterator> faces() noexcept;
    MDV_NODISCARD boost::iterator_range<ConstFaceIterator> faces() const noexcept;
    MDV_NODISCARD boost::iterator_range<ConstFaceIterator> cfaces() const noexcept;


public:
    friend class Vertex;

    // Members
    LoggerPtr_t           _logger;
    gsl::owner<CgalData*> _cgal_data{nullptr};
    std::filesystem::path _file_path;

    Eigen::Matrix<double, 3, Eigen::Dynamic> _v_mat;
    Eigen::Matrix<long, 3, Eigen::Dynamic>   _f_mat;

    std::vector<std::array<VertexIndex_t, 3>> _neighbouring_faces;

    friend class Face;
    friend class Vertex;
    friend class Point;
};


}  // namespace mdv::mesh


#endif  // MDV_MESH_CLASS_HPP
