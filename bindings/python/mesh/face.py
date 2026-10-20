from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray

from . import _mesh_impl as _impl

if TYPE_CHECKING:
    from .mesh import Mesh


class Face:
    """A face index paired with its owning Python mesh."""

    def __init__(self, mesh: Mesh, face_id: int):
        self._mesh = mesh
        self._id = face_id

    @property
    def id(self) -> int:
        """The face index."""
        return self._id

    @property
    def normal(self) -> NDArray[np.float64]:
        """The unit normal of this face."""
        return np.asarray(
            _impl.face_normal(self._mesh._mesh_impl, self._id)
        ).reshape(-1)

    @property
    def mesh(self) -> Mesh:
        """The original Python mesh owner."""
        return self._mesh

    def describe(self) -> str:
        """Return the C++ face description."""
        return _impl.face_describe(self._mesh._mesh_impl, self._id)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Face):
            return NotImplemented
        return self._mesh is other._mesh and self._id == other._id

    def __hash__(self) -> int:
        return hash((id(self._mesh), self._id))
