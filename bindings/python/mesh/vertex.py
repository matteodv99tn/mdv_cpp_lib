from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray

try:
    from ._mesh_impl import Vertex as _VertexImpl  # type: ignore[import-not-found]
except ImportError:  # pragma: no cover - generated at build time

    class _VertexImpl:  # type: ignore[no-redef]
        def position(self) -> NDArray[np.float64]:
            raise RuntimeError("SWIG bindings are not built yet.")

        def normal(self) -> NDArray[np.float64]:
            raise RuntimeError("SWIG bindings are not built yet.")

        def id(self) -> int:
            raise RuntimeError("SWIG bindings are not built yet.")

        def describe(self) -> str:
            raise RuntimeError("SWIG bindings are not built yet.")

        def total_curvature(self) -> float:
            raise RuntimeError("SWIG bindings are not built yet.")

        def gauss_curvature(self) -> float:
            raise RuntimeError("SWIG bindings are not built yet.")


class Vertex:
    """Python wrapper for a mesh vertex.

    Attributes
    ----------
    position : np.ndarray
        3D position of the vertex.
    normal : np.ndarray
        Normal vector at the vertex.
    id : int
        Unique identifier of the vertex.
    """

    def __init__(self, vertex_impl: _VertexImpl):
        """Initialize a Vertex wrapper.

        Parameters
        ----------
        vertex_impl : _VertexImpl
            The underlying SWIG vertex object.
        """
        self._vertex_impl = vertex_impl

    @property
    def position(self) -> NDArray[np.float64]:
        """Get the 3D position of the vertex.

        Returns
        -------
        np.ndarray
            3D position vector ``[x, y, z]``.
        """
        return np.asarray(self._vertex_impl.position()).reshape(-1)

    @property
    def normal(self) -> NDArray[np.float64]:
        """Get the normal vector at the vertex.

        Returns
        -------
        np.ndarray
            Normal vector ``[x, y, z]``.
        """
        return np.asarray(self._vertex_impl.normal()).reshape(-1)

    @property
    def id(self) -> int:
        """Get the unique identifier of the vertex.

        Returns
        -------
        int
            Vertex identifier.
        """
        return self._vertex_impl.id()

    def describe(self) -> str:
        """Get a string description of the vertex.

        Returns
        -------
        str
            Description of the vertex.
        """
        return self._vertex_impl.describe()

    def total_curvature(self) -> float:
        """Compute total curvature at the vertex.

        This is the total curvature defined in
        "Straightest geodesics on polyhedral surfaces" (eq. 5).

        Returns
        -------
        float
            Total curvature.
        """
        return self._vertex_impl.total_curvature()

    def gauss_curvature(self) -> float:
        """Compute Gaussian curvature at the vertex.

        This is the Gauss curvature defined in
        "Straightest geodesics on polyhedral surfaces" (eq. 6).

        Returns
        -------
        float
            Gauss curvature.
        """
        return self._vertex_impl.gauss_curvature()
