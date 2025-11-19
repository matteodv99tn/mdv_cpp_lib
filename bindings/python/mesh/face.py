from ._mesh_impl import Face as _FaceImpl
from typing import List
import numpy as np

class Face:
    """
    Python wrapper for mesh face.
    
    This class represents a face in a mesh, containing identification
    and normal information.
    
    Attributes
    ----------
    id : int
        Unique identifier of the face
    normal : np.ndarray
        Normal vector of the face
    """

    def __init__(self, face_impl: _FaceImpl):
        """
        Initialize a Face wrapper.
        
        Parameters
        ----------
        face_impl : _FaceImpl
            The underlying SWIG face object
        """
        self._face_impl = face_impl

    @property
    def id(self) -> int:
        """
        Get the unique identifier of the face.
        
        Returns
        -------
        int
            Face identifier
        """
        return self._face_impl.id()

    @property
    def normal(self) -> np.ndarray:
        """
        Get the normal vector of the face.
        
        Returns
        -------
        np.ndarray
            Normal vector [x, y, z]
        """
        return self._face_impl.normal()

    def describe(self) -> str:
        """
        Get a string description of the face.
        
        Returns
        -------
        str
            Description of the face
        """
        return self._face_impl.describe()
