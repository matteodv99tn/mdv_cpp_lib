from ._mesh_impl import Point as _PointImpl
from typing import List
import numpy as np

class Point:
    """
    Python wrapper for mesh point.
    
    This class represents a point on a mesh, which can be defined either
    by its position in 3D space or by its location on a face using UV coordinates.
    
    Attributes
    ----------
    position : np.ndarray
        3D position of the point
    face : Face
        Face on which the point is located
    """

    def __init__(self, point_impl: _PointImpl):
        """
        Initialize a Point wrapper.
        
        Parameters
        ----------
        point_impl : _PointImpl
            The underlying SWIG point object
        """
        self._point_impl = point_impl

    @property
    def position(self) -> np.ndarray:
        """
        Get the 3D position of the point.
        
        Returns
        -------
        np.ndarray
            3D position vector [x, y, z]
        """
        return self._point_impl.position()

    @property
    def face(self) -> 'Face':
        """
        Get the face on which the point is located.
        
        Returns
        -------
        Face
            The face containing this point
        """
        from .mesh_face import Face
        return Face(self._point_impl.face())

    def describe(self) -> str:
        """
        Get a string description of the point.
        
        Returns
        -------
        str
            Description of the point
        """
        return self._point_impl.describe()

    @staticmethod
    def from_cartesian(mesh: 'Mesh', position: np.ndarray) -> 'Point':
        """
        Create a point from cartesian coordinates.
        
        Parameters
        ----------
        mesh : Mesh
            The mesh to which the point belongs
        position : np.ndarray
            3D position vector [x, y, z]
            
        Returns
        -------
        Point
            New point created from cartesian coordinates
        """
        return Point(_PointImpl.from_cartesian(mesh._mesh_impl, position))

    @staticmethod
    def random(mesh: 'Mesh') -> 'Point':
        """
        Create a random point on the mesh.
        
        Parameters
        ----------
        mesh : Mesh
            The mesh on which to generate the random point
            
        Returns
        -------
        Point
            Random point on the mesh
        """
        from .mesh import Mesh
        return Point(_PointImpl.random(mesh._mesh_impl))
