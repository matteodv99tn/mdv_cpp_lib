from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

try:
    from ._mesh_impl import TangentVector as _TangentVectorImpl  # type: ignore[import-not-found]
except ImportError:  # pragma: no cover - generated at build time

    class _TangentVectorImpl:  # type: ignore[no-redef]
        def __init__(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        @staticmethod
        def from_ambient_vector(*_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        @staticmethod
        def from_tip_position(*_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        @staticmethod
        def unit_random(*_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        def tip(self):
            raise RuntimeError("SWIG bindings are not built yet.")

        def cartesian_vector(self):
            raise RuntimeError("SWIG bindings are not built yet.")

        def scale(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        def normalise(self):
            raise RuntimeError("SWIG bindings are not built yet.")

        def normalised(self):
            raise RuntimeError("SWIG bindings are not built yet.")

        def application_point(self):
            raise RuntimeError("SWIG bindings are not built yet.")

        def type(self):
            raise RuntimeError("SWIG bindings are not built yet.")


from ._validation import as_vector
from .point import Point


class TangentVector:
    """Python wrapper for a mesh tangent vector.

    Parameters
    ----------
    application_point : Point
        Point on the mesh where the vector is applied.
    vector : array_like, shape (3,)
        Cartesian vector in 3D.
    """

    def __init__(self, application_point: Point, vector: ArrayLike):
        vector = as_vector(vector, size=3, name="vector")
        self._tv_impl = _TangentVectorImpl(
            application_point._point_impl,  # type: ignore[attr-defined]
            vector,
        )

    @staticmethod
    def from_ambient_vector(
        application_point: Point, vector: ArrayLike
    ) -> "TangentVector":
        """Project an ambient vector onto the surface tangent space.

        Parameters
        ----------
        application_point : Point
            Point on the mesh where the vector is applied.
        vector : array_like, shape (3,)
            Cartesian vector in 3D.

        Returns
        -------
        TangentVector
            Tangent vector on the mesh.
        """
        vector = as_vector(vector, size=3, name="vector")
        tv_impl = _TangentVectorImpl.from_ambient_vector(
            application_point._point_impl,  # type: ignore[attr-defined]
            vector,
        )
        return TangentVector.from_impl(tv_impl)

    @staticmethod
    def from_tip_position(application_point: Point, tip: ArrayLike) -> "TangentVector":
        """Create a tangent vector from a tip position in 3D.

        Parameters
        ----------
        application_point : Point
            Point on the mesh where the vector is applied.
        tip : array_like, shape (3,)
            Tip position in 3D.

        Returns
        -------
        TangentVector
            Tangent vector on the mesh.
        """
        tip_vec = as_vector(tip, size=3, name="tip")
        tv_impl = _TangentVectorImpl.from_tip_position(
            application_point._point_impl,  # type: ignore[attr-defined]
            tip_vec,
        )
        return TangentVector.from_impl(tv_impl)

    @staticmethod
    def unit_random(application_point: Point) -> "TangentVector":
        """Generate a random unit tangent vector at a point.

        Parameters
        ----------
        application_point : Point
            Point on the mesh where the vector is applied.

        Returns
        -------
        TangentVector
            Unit tangent vector.
        """
        tv_impl = _TangentVectorImpl.unit_random(
            application_point._point_impl  # type: ignore[attr-defined]
        )
        return TangentVector.from_impl(tv_impl)

    @classmethod
    def from_impl(cls, tv_impl: _TangentVectorImpl) -> "TangentVector":
        obj = cls.__new__(cls)
        obj._tv_impl = tv_impl
        return obj

    def tip(self) -> NDArray[np.float64]:
        """Return the tip position (application point + vector).

        Returns
        -------
        np.ndarray
            Tip position as a 1D vector of shape (3,).
        """
        return np.asarray(self._tv_impl.tip()).reshape(-1)

    def cartesian_vector(self) -> NDArray[np.float64]:
        """Return the ambient Cartesian vector.

        Returns
        -------
        np.ndarray
            Cartesian vector as a 1D vector of shape (3,).
        """
        return np.asarray(self._tv_impl.cartesian_vector()).reshape(-1)

    def scale(self, factor: float) -> None:
        """Scale the vector magnitude.

        Parameters
        ----------
        factor : float
            Scaling factor.
        """
        self._tv_impl.scale(float(factor))

    def normalise(self) -> None:
        """Normalise the vector in place."""
        self._tv_impl.normalise()

    def normalised(self) -> "TangentVector":
        """Return a normalized copy of this vector."""
        return TangentVector.from_impl(self._tv_impl.normalised())

    def application_point(self) -> Point:
        """Return the application point for this vector."""
        return Point(self._tv_impl.application_point())

    def type(self) -> int:
        """Return the tangent vector type enum value."""
        return int(self._tv_impl.type())
