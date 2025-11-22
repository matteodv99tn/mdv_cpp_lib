#include "mdv/mesh/vertex.hpp"

#include <CGAL/boost/graph/properties.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <Eigen/Dense>
#include <fmt/format.h>
#include <iterator>

#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/utils/logging_extras.hpp"

// \cond DOXYGEN_IGNORE
using mdv::mesh::Vertex;

// \endcond

std::string
Vertex::describe() const {
    if (!is_valid()) return "Invalid Vertex";

    return fmt::format(
            "Vertex ID #{} (position: {}) on mesh '{}'",
            id(),
            eigen_to_str(position()),
            mesh().name()
    );
}
