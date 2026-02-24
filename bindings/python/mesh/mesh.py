from __future__ import annotations

from typing import Optional

import numpy as np
import pymeshlab  # type: ignore[import-not-found]
import pyvista  # type: ignore[import-not-found]
from numpy.typing import ArrayLike, NDArray

try:
    from ._mesh_impl import (  # type: ignore[import-not-found]
        Mesh as _MeshImpl,
        extract_normal_bounded_surface,
        fill_holes,
        load_from_file,
        mesh_directory,
    )
except ImportError:  # pragma: no cover - generated at build time

    class _MeshImpl:  # type: ignore[no-redef]
        def __init__(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        def name(self) -> str:
            raise RuntimeError("SWIG bindings are not built yet.")

        def num_vertices(self) -> int:
            raise RuntimeError("SWIG bindings are not built yet.")

        def num_faces(self) -> int:
            raise RuntimeError("SWIG bindings are not built yet.")

        def vertex(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        def face(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        def build_geodesic(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        def get_vertex_matrix(self) -> NDArray[np.float64]:
            raise RuntimeError("SWIG bindings are not built yet.")

        def get_face_matrix_double(self) -> NDArray[np.float64]:
            raise RuntimeError("SWIG bindings are not built yet.")

        def closest_vertex(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

    def load_from_file(*_args, **_kwargs):
        raise RuntimeError("SWIG bindings are not built yet.")

    def mesh_directory() -> str:
        raise RuntimeError("SWIG bindings are not built yet.")

    def extract_normal_bounded_surface(*_args, **_kwargs):
        raise RuntimeError("SWIG bindings are not built yet.")

    def fill_holes(*_args, **_kwargs):
        raise RuntimeError("SWIG bindings are not built yet.")


from .face import Face
from .geodesic import Geodesic
from .point import Point
from .vertex import Vertex
from . import _mesh_impl as _impl  # type: ignore[import-not-found]
from ._validation import as_matrix, as_vector


class Mesh:
    """Python wrapper for the mesh data structure.

    This class provides access to mesh geometry, topological data, and
    geodesic operations.

    Attributes
    ----------
    name : str
        Name of the mesh.
    num_vertices : int
        Number of vertices in the mesh.
    num_faces : int
        Number of faces in the mesh.
    """

    def __init__(
        self,
        vertices: Optional[ArrayLike] = None,
        faces: Optional[ArrayLike] = None,
        mesh_impl: Optional[_MeshImpl] = None,
    ):
        """Initialize a Mesh wrapper.

        This constructor supports two initialization methods:

        1) From vertices and faces arrays.
        2) From an existing SWIG mesh object (``mesh_impl``).

        Parameters
        ----------
        vertices : array_like, optional
            Vertex coordinates with shape ``(nv, 3)``.
        faces : array_like, optional
            Face indices with shape ``(nf, 3)``.
        mesh_impl : _MeshImpl, optional
            The underlying SWIG mesh object.

        Raises
        ------
        RuntimeError
            If ``vertices`` or ``faces`` are missing when constructing from arrays.

        Notes
        -----
        When providing vertices and faces arrays, the mesh is created using
        ``pymeshlab`` and loaded from a temporary OFF file.
        """
        import tempfile

        if mesh_impl is not None:
            self._mesh_impl = mesh_impl
            return

        if (faces is None) or (vertices is None):
            raise RuntimeError(
                "To construct a mesh, provide (nv, 3) vertices and (nf, 3) faces."
            )

        vertex_matrix = as_matrix(vertices, shape=(-1, 3), name="vertices")
        face_matrix = np.asarray(faces)
        if face_matrix.ndim != 2 or face_matrix.shape[1] != 3:
            raise ValueError(f"faces must have shape (nf, 3); got {face_matrix.shape}.")
        if not np.issubdtype(face_matrix.dtype, np.integer):
            face_matrix = face_matrix.astype(np.int32)

        pymesh = pymeshlab.Mesh(  # type: ignore[attr-defined]
            vertex_matrix=vertex_matrix,
            face_matrix=face_matrix,
        )
        meshset = pymeshlab.MeshSet()  # type: ignore[attr-defined]
        meshset.add_mesh(pymesh)

        with tempfile.NamedTemporaryFile(suffix=".off", delete=False) as tmp:
            meshset.save_current_mesh(
                tmp.name,
                save_vertex_color=False,
                save_vertex_normal=False,
                save_face_color=False,
                save_polygonal=False,
            )
            self._mesh_impl = load_from_file(tmp.name)

    def scale(self, scaling: float):
        """
        Scales the mesh by a given scaling factor

        Parameters
        ----------
        scaling: float
            The scaling factor coefficient
        """
        self._mesh_impl.scale(scaling)

    @staticmethod
    def load_from_file(file_path: str) -> "Mesh":
        """Load a mesh from a file.

        Parameters
        ----------
        file_path : str
            Path to the mesh file.

        Returns
        -------
        Mesh
            Loaded mesh object.
        """
        return Mesh(mesh_impl=load_from_file(file_path))

    @staticmethod
    def mesh_directory() -> str:
        """Return the directory where mesh files are stored.

        Returns
        -------
        str
            Path to the mesh directory.
        """
        return mesh_directory()

    def transform(self, matrix: ArrayLike) -> None:
        """Apply an affine transform to the mesh.

        Parameters
        ----------
        matrix : array_like, shape (4, 4)
            Homogeneous transform matrix.
        """
        transform = np.asarray(matrix, dtype=np.float64)
        if transform.shape != (4, 4):
            raise ValueError("matrix must have shape (4, 4).")
        from . import _mesh_impl as _impl  # type: ignore[import-not-found]

        _impl.apply_transform(self._mesh_impl, transform)  # type: ignore[attr-defined]

    @staticmethod
    def cube_angle() -> "Mesh":
        """Create a unit cube corner mesh.

        The mesh represents three faces of a unit cube sharing the vertex at
        ``(1, 1, 1)``.

        Returns
        -------
        Mesh
            A new mesh representing the cube corner.
        """
        vertices = np.array(
            [
                [1.0, 0.0, 0.0],  # vertex 0
                [1.0, 1.0, 0.0],  # vertex 1
                [0.0, 1.0, 0.0],  # vertex 2
                [0.0, 1.0, 1.0],  # vertex 3
                [1.0, 1.0, 1.0],  # vertex 4
                [1.0, 0.0, 1.0],  # vertex 5
                [0.0, 0.0, 1.0],  # vertex 6
            ],
            dtype=np.float64,
        )
        faces = np.array(
            [[0, 1, 4], [0, 4, 5], [1, 2, 4], [2, 3, 4], [4, 3, 6], [4, 6, 5]],
            dtype=np.int32,
        )
        return Mesh(vertices=vertices, faces=faces)

    @property
    def name(self) -> str:
        """Return the name of the mesh.

        Returns
        -------
        str
            Name of the mesh.
        """
        return self._mesh_impl.name()

    @property
    def num_vertices(self) -> int:
        """Return the number of vertices in the mesh.

        Returns
        -------
        int
            Number of vertices.
        """
        return self._mesh_impl.num_vertices()

    @property
    def num_faces(self) -> int:
        """Return the number of faces in the mesh.

        Returns
        -------
        int
            Number of faces.
        """
        return self._mesh_impl.num_faces()

    def get_vertex_matrix(self) -> NDArray[np.float64]:
        """Return the vertex matrix of the mesh.

        Returns
        -------
        np.ndarray
            Matrix of shape ``(N, 3)`` with vertex positions.
        """
        return np.asarray(self._mesh_impl.get_vertex_matrix())

    def get_face_matrix(self) -> NDArray[np.int32]:
        """Return the face index matrix of the mesh.

        Returns
        -------
        np.ndarray
            Matrix of shape ``(N, 3)`` with face indices ``[v0, v1, v2]``.

        Notes
        -----
        The SWIG binding exposes faces as doubles; this wrapper converts them to
        ``int32`` for typical Python usage.
        """
        res = np.asarray(self._mesh_impl.get_face_matrix_double())
        return res.astype(np.int32)

    def vertex(self, id: int) -> Vertex:
        """Return a vertex by its identifier.

        Parameters
        ----------
        id : int
            Vertex identifier.

        Returns
        -------
        Vertex
            Vertex with the specified identifier.
        """
        return Vertex(self._mesh_impl.vertex(id))

    def face(self, id: int) -> Face:
        """Return a face by its identifier.

        Parameters
        ----------
        id : int
            Face identifier.

        Returns
        -------
        Face
            Face with the specified identifier.
        """
        return Face(self._mesh_impl.face(id))

    def random_face(self) -> Face:
        """Return a random face from the mesh."""
        return Face(self._mesh_impl.random_face())  # type: ignore[attr-defined]

    @property
    def faces(self) -> list[Face]:
        """Return all faces in the mesh.

        Returns
        -------
        list[Face]
            Face objects for the mesh.
        """
        return [self.face(i) for i in range(self.num_faces)]

    @property
    def vertices(self) -> list[Vertex]:
        """Return all vertices in the mesh.

        Returns
        -------
        list[Vertex]
            Vertex objects for the mesh.
        """
        return [self.vertex(i) for i in range(self.num_vertices)]

    def build_geodesic(self, from_point: Point, to_point: Point) -> Geodesic:
        """Build a geodesic path between two mesh points.

        Parameters
        ----------
        from_point : Point
            Starting point of the geodesic.
        to_point : Point
            Ending point of the geodesic.

        Returns
        -------
        Geodesic
            Geodesic path between the two points.
        """
        return Geodesic(
            self._mesh_impl.build_geodesic(
                from_point._point_impl,  # type: ignore[attr-defined]
                to_point._point_impl,  # type: ignore[attr-defined]
            )
        )

    def midpoint_subdivide(self, iterations: int = 1) -> "Mesh":
        """Subdivide the mesh using midpoint subdivision.

        Parameters
        ----------
        iterations : int, default=1
            Number of subdivision iterations to perform.

        Returns
        -------
        Mesh
            Subdivided mesh.

        Notes
        -----
        Each iteration increases the number of faces approximately by a factor of 4.
        """
        import tempfile

        meshset = self._create_meshset()
        meshset.meshing_surface_subdivision_midpoint(iterations=iterations)

        with tempfile.NamedTemporaryFile(suffix=".off", delete=False) as tmp:
            meshset.save_current_mesh(
                tmp.name,
                save_vertex_color=False,
                save_vertex_normal=False,
                save_face_color=False,
                save_polygonal=False,
            )
            newmesh = Mesh.load_from_file(tmp.name)
            return newmesh

    def to_pyvista(self) -> pyvista.PolyData:
        """Convert the mesh to a PyVista PolyData object.

        Returns
        -------
        pyvista.PolyData
            Triangular mesh representation usable with PyVista.
        """
        faces = self.get_face_matrix()
        vertices = self.get_vertex_matrix()
        return pyvista.make_tri_mesh(vertices, faces)

    def _create_meshset(self) -> "pymeshlab.MeshSet":  # type: ignore[attr-defined]
        """Create a pymeshlab MeshSet from this mesh.

        Returns
        -------
        MeshSet
            MeshSet containing this mesh.
        """
        pymesh = pymeshlab.Mesh(  # type: ignore[attr-defined]
            vertex_matrix=self.get_vertex_matrix(),
            face_matrix=self.get_face_matrix(),
        )
        meshset = pymeshlab.MeshSet()  # type: ignore[attr-defined]
        meshset.add_mesh(pymesh)
        return meshset

    def closest_vertex(self, point: ArrayLike) -> Vertex:
        """Find the closest vertex to a 3D point.

        Parameters
        ----------
        point : array_like, shape (3,)
            3D point coordinates ``[x, y, z]``.

        Returns
        -------
        Vertex
            Closest vertex on the mesh.
        """
        point_array = as_vector(point, size=3, name="point")
        vertex_impl = self._mesh_impl.closest_vertex(point_array)
        return Vertex(vertex_impl)

    @staticmethod
    def extract_normal_bounded_surface(
        mesh: "Mesh",
        point: Point,
        max_normal_angle: float = 90.0,
    ) -> "Mesh":
        """Extract a connected submesh by bounding normal deviation.

        Starting from a seed point, this function propagates across adjacent faces
        and keeps faces whose normals stay within the specified angle of the seed
        face normal.

        Parameters
        ----------
        mesh : Mesh
            Source mesh.
        point : Point
            Seed point on the mesh.
        max_normal_angle : float, default=90.0
            Maximum allowed normal deviation in degrees.

        Returns
        -------
        Mesh
            Extracted submesh.
        """
        mesh_impl = extract_normal_bounded_surface(
            mesh._mesh_impl,  # type: ignore[attr-defined]
            point._point_impl,  # type: ignore[attr-defined]
            max_normal_angle,
        )
        return Mesh(mesh_impl=mesh_impl)

    @staticmethod
    def fill_holes(mesh: "Mesh") -> "Mesh":
        """Returns a new mesh which fills all internal holes of the provided mesh

        Parameters
        ----------
        mesh : Mesh
            Source mesh.

        Returns
        -------
        Mesh
            Extracted submesh.
        """
        mesh_impl = fill_holes(
            mesh._mesh_impl,  # type: ignore[attr-defined]
        )
        return Mesh(mesh_impl=mesh_impl)
