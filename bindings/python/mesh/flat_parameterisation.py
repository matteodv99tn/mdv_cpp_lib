from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

try:
    from ._mesh_impl import FlatParameterisation as _FlatParameterisationImpl  # type: ignore[import-not-found]
except ImportError:  # pragma: no cover - generated at build time

    class _FlatParameterisationImpl:  # type: ignore[no-redef]
        def __init__(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        def is_one_to_one_mapping(self) -> bool:
            raise RuntimeError("SWIG bindings are not built yet.")

        def project(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        def retrieve(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        def is_inside_mesh(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        def min_uv(self):
            raise RuntimeError("SWIG bindings are not built yet.")

        def max_uv(self):
            raise RuntimeError("SWIG bindings are not built yet.")


from .mesh import Mesh
from .point import Point
from ._validation import as_vector


class FlatParameterisation:
    """
    Python wrapper for 2D parameterization of surface meshes.

    Provides UV mapping for projecting 3D surface points to a planar domain and
    lifting them back, which is useful for visualization, learning in 2D, and
    building surface-conditioned policies.

    Parameters
    ----------
    mesh : Mesh
        The mesh object to parameterize

    Notes
    -----
    This class provides UV mapping functionality for surface meshes. It allows
    projecting 3D points on the mesh surface to 2D UV coordinates and lifting
    2D UV coordinates back to 3D points on the mesh surface.
    """

    def __init__(self, mesh: Mesh):
        """Initialize a FlatParameterisation wrapper.

        Parameters
        ----------
        mesh : Mesh
            Mesh to parameterize.
        """
        self._parameterisation_impl = _FlatParameterisationImpl(mesh._mesh_impl)

    def is_one_to_one_mapping(self) -> bool:
        """
        Check if the parameterization is one-to-one.

        Returns
        -------
        bool
            True if the mapping is one-to-one, False otherwise

        Notes
        -----
        A one-to-one mapping means each point in the 2D domain maps to exactly
        one point on the mesh surface, and vice versa. This is important for
        certain applications where bijective mappings are required.
        """
        return self._parameterisation_impl.is_one_to_one_mapping()

    def project(self, point: Point) -> NDArray[np.float64]:
        """
        Project a surface point to UV coordinates.

        Parameters
        ----------
        point : Point
            Point on the mesh surface to project

        Returns
        -------
        np.ndarray
            UV coordinates as a 2D numpy array [u, v]

        Notes
        -----
        This method transforms a 3D point on the mesh surface to 2D UV coordinates
        in the parameterized domain. The UV coordinates represent the planar
        projection of the surface point.
        """
        return np.asarray(
            self._parameterisation_impl.project(point._point_impl)
        ).reshape(-1)

    def retrieve(self, uv: ArrayLike) -> Point:
        """
        Lift a UV point back to the mesh surface.

        Parameters
        ----------
        uv : np.ndarray
            UV coordinates as a 2D numpy array [u, v]

        Returns
        -------
        Point
            Point on the mesh surface

        Notes
        -----
        This method transforms 2D UV coordinates back to a 3D point on the mesh
        surface. This is the inverse operation of the project method.
        """
        uv_vec = as_vector(uv, size=2, name="uv")
        point_impl = self._parameterisation_impl.retrieve(uv_vec)
        return Point(point_impl)

    def is_inside_mesh(self, uv: ArrayLike) -> bool:
        """
        Check if a UV point lies inside the parameterized mesh domain.

        Parameters
        ----------
        uv : np.ndarray
            UV coordinates as a 2D numpy array [u, v]

        Returns
        -------
        bool
            True if the UV coordinates are inside the mesh domain, False otherwise

        Notes
        -----
        This method determines whether the given UV coordinates fall within the
        valid parameterization domain of the mesh. Points outside this domain
        may not have meaningful mappings back to the mesh surface.
        """
        uv_vec = as_vector(uv, size=2, name="uv")
        return self._parameterisation_impl.is_inside_mesh(uv_vec)

    def min_uv(self) -> NDArray[np.float64]:
        """
        Get the minimum UV coordinates in the parameterization.

        Returns
        -------
        np.ndarray
            Minimum UV values as a 2D numpy array [u_min, v_min]

        Notes
        -----
        Returns the lower bounds of the UV parameterization domain. These values
        define the extent of the 2D parameterization space.
        """
        return np.asarray(self._parameterisation_impl.min_uv()).reshape(-1)

    def max_uv(self) -> NDArray[np.float64]:
        """
        Get the maximum UV coordinates in the parameterization.

        Returns
        -------
        np.ndarray
            Maximum UV values as a 2D numpy array [u_max, v_max]

        Notes
        -----
        Returns the upper bounds of the UV parameterization domain. These values
        define the extent of the 2D parameterization space.
        """
        return np.asarray(self._parameterisation_impl.max_uv()).reshape(-1)
