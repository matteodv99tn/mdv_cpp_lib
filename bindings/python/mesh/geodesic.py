from ._mesh_impl import Geodesic as _GeodesicImpl
from typing import List
import numpy as np

class Geodesic:
    """ 
    Python wrapper of a geodesic
    """

    def __init__(self, geodesic_impl: _GeodesicImpl):
        self._geodesic_impl = geodesic_impl

    def length(self) -> float:
        """
        Returns the length of the geodesic
        """
        from ._mesh_impl import length
        return length(self._geodesic_impl)

    def to_list(self) -> List[np.ndarray]:
        """
        Convert geodesic to list of numpy arrays.
        
        Returns
        -------
        List[np.ndarray]
            List of 3D points representing the geodesic path
        """
        result = []
        for i in range(len(self._geodesic_impl)):
            result.append(self._geodesic_impl[i])
        return result
