from ._mesh_impl import Vertex as _VertexImpl
from typing import List
import numpy as np

class Vertex:
    """
    Python wrapper for mesh vertex.
    
    This class represents a vertex in a mesh, containing position, normal,
    and identification information.
    
    Attributes
    ----------
    position : np.ndarray
        3D position of the vertex
    normal : np.ndarray
        Normal vector at the vertex
    id : int
        Unique identifier of the vertex
    """

    def __init__(self, vertex_impl: _VertexImpl):
        """
        Initialize a Vertex wrapper.

        Parameters
        ----------
        vertex_impl : _VertexImpl
            The underlying SWIG vertex object
        """
        self._vertex_impl = vertex_impl

    @property
    def position(self) -> np.ndarray:
        """
        Get the 3D position of the vertex.
        
        Returns
        -------
        np.ndarray
            3D position vector [x, y, z]
        """
        return self._vertex_impl.position()

    @property
    def normal(self) -> np.ndarray:
        """
        Get the normal vector at the vertex.
        
        Returns
        -------
        np.ndarray
            Normal vector [x, y, z]
        """
        return self._vertex_impl.normal()

    @property
    def id(self) -> int:
        """
        Get the unique identifier of the vertex.
        
        Returns
        -------
        int
            Vertex identifier
        """
        return self._vertex_impl.id()

    def describe(self) -> str:
        """
        Get a string description of the vertex.
        
        Returns
        -------
        str
            Description of the vertex
        """
        return self._vertex_impl.describe()
