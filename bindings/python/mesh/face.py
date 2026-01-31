from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

try:
    from ._mesh_impl import Face as _FaceImpl  # type: ignore[import-not-found]
except ImportError:  # pragma: no cover - generated at build time

    class _FaceImpl:  # type: ignore[no-redef]
        def id(self) -> int:
            raise RuntimeError("SWIG bindings are not built yet.")

        def normal(self) -> NDArray[np.float64]:
            raise RuntimeError("SWIG bindings are not built yet.")

        def describe(self) -> str:
            raise RuntimeError("SWIG bindings are not built yet.")


class Face:
    """Python wrapper for a mesh face.

    Attributes
    ----------
    id : int
        Unique identifier of the face.
    normal : np.ndarray
        Normal vector of the face.
    """

    def __init__(self, face_impl: _FaceImpl):
        """Initialize a Face wrapper.

        Parameters
        ----------
        face_impl : _FaceImpl
            The underlying SWIG face object.
        """
        self._face_impl = face_impl

    @property
    def id(self) -> int:
        """Get the unique identifier of the face.

        Returns
        -------
        int
            Face identifier.
        """
        return self._face_impl.id()

    @property
    def normal(self) -> NDArray[np.float64]:
        """Get the normal vector of the face.

        Returns
        -------
        np.ndarray
            Normal vector ``[x, y, z]``.
        """
        return self._face_impl.normal()

    def describe(self) -> str:
        """Get a string description of the face.

        Returns
        -------
        str
            Description of the face.
        """
        return self._face_impl.describe()
