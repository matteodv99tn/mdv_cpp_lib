from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray

from . import _mesh_impl as _impl

if TYPE_CHECKING:
    from .mesh import Mesh


class Vertex:
    """A vertex index paired with its owning Python mesh."""

    def __init__(self, mesh: Mesh, vertex_id: int):
        self._mesh = mesh
        self._id = vertex_id

    @property
    def mesh(self) -> Mesh:
        """The mesh owning this vertex."""
        return self._mesh

    @property
    def position(self) -> NDArray[np.float64]:
        """The 3D position of the vertex."""
        return np.asarray(_impl.vertex_position(self._mesh._mesh_impl, self._id)).reshape(-1)

    @property
    def normal(self) -> NDArray[np.float64]:
        """The precomputed vertex normal."""
        return np.asarray(_impl.vertex_normal(self._mesh._mesh_impl, self._id)).reshape(-1)

    @property
    def id(self) -> int:
        return self._id

    def describe(self) -> str:
        return _impl.vertex_describe(self._mesh._mesh_impl, self._id)

    def total_curvature(self) -> float:
        """Sum of incident angles, in radians (legacy Python API name)."""
        return _impl.total_angle(self._mesh._mesh_impl, self._id)

    def gauss_curvature(self) -> float:
        """Gaussian curvature, in radians."""
        return _impl.gauss_curvature(self._mesh._mesh_impl, self._id)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Vertex):
            return NotImplemented
        return self._mesh is other._mesh and self._id == other._id

    def __hash__(self) -> int:
        return hash((id(self._mesh), self._id))
