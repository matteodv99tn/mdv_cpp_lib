from .vertex import Vertex
from .face import Face
from .point import Point
from .geodesic import Geodesic
from ._mesh_impl import Mesh as _MeshImpl, load_from_file, mesh_directory
from typing import List
import numpy as np

class Mesh:
    """
    Python wrapper for mesh data structure.
    
    This class provides access to mesh data including vertices, faces,
    and geometric operations like geodesic path computation.
    
    Attributes
    ----------
    name : str
        Name of the mesh
    num_vertices : int
        Number of vertices in the mesh
    num_faces : int
        Number of faces in the mesh
    """

    def __init__(self, mesh_impl: _MeshImpl):
        """
        Initialize a Mesh wrapper.
        
        Parameters
        ----------
        mesh_impl : _MeshImpl
            The underlying SWIG mesh object
        """
        self._mesh_impl = mesh_impl

    @property
    def num_vertices(self) -> int:
        """
        Get the number of vertices in the mesh.
        
        Returns
        -------
        int
            Number of vertices
        """
        return self._mesh_impl.num_vertices()

    @property
    def num_faces(self) -> int:
        """
        Get the number of faces in the mesh.
        
        Returns
        -------
        int
            Number of faces
        """
        return self._mesh_impl.num_faces()

    def vertex(self, id: int) -> Vertex:
        """
        Get a vertex by its identifier.
        
        Parameters
        ----------
        id : int
            Vertex identifier
            
        Returns
        -------
        Vertex
            Vertex with the specified identifier
        """
        return Vertex(self._mesh_impl.vertex(id))

    def face(self, id: int) -> Face:
        """
        Get a face by its identifier.
        
        Parameters
        ----------
        id : int
            Face identifier
            
        Returns
        -------
        Face
            Face with the specified identifier
        """
        return Face(self._mesh_impl.face(id))

    def build_geodesic(self, from_point: Point, to_point: Point) -> Geodesic:
        """
        Build a geodesic path between two points on the mesh.
        
        Parameters
        ----------
        from_point : Point
            Starting point of the geodesic
        to_point : Point
            Ending point of the geodesic
            
        Returns
        -------
        Geodesic
            Geodesic path between the two points
        """
        return Geodesic(
            self._mesh_impl.build_geodesic(from_point._point_impl, to_point._point_impl))

    @staticmethod
    def load_from_file(file_path: str) -> 'Mesh':
        """
        Load a mesh from a file.
        
        Parameters
        ----------
        file_path : str
            Path to the mesh file
            
        Returns
        -------
        Mesh
            Loaded mesh object
        """
        return Mesh(load_from_file(file_path))

    @staticmethod
    def mesh_directory() -> str:
        """
        Get the directory where mesh files are stored.
        
        Returns
        -------
        str
            Path to the mesh directory
        """
        return mesh_directory()
