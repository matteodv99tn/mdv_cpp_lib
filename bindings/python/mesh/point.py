from __future__ import annotations

from typing import TYPE_CHECKING, Union

import numpy as np
from numpy.typing import ArrayLike, NDArray

try:
    from ._mesh_impl import Point as _PointImpl  # type: ignore[import-not-found]
except ImportError:  # pragma: no cover - generated at build time

    class _PointImpl:  # type: ignore[no-redef]
        def __init__(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        @staticmethod
        def from_cartesian(*_args, **_kwargs) -> "_PointImpl":
            raise RuntimeError("SWIG bindings are not built yet.")

        @staticmethod
        def random(*_args, **_kwargs) -> "_PointImpl":
            raise RuntimeError("SWIG bindings are not built yet.")

        def position(self) -> NDArray[np.float64]:
            raise RuntimeError("SWIG bindings are not built yet.")

        def face(self):
            raise RuntimeError("SWIG bindings are not built yet.")

        def describe(self) -> str:
            raise RuntimeError("SWIG bindings are not built yet.")


from .vertex import Vertex
from . import _mesh_impl as _impl
from ._validation import as_vector

if TYPE_CHECKING:
    from .face import Face
    from .mesh import Mesh


class Point:
    """Python wrapper for a point on a mesh.

    A point can be constructed from a mesh vertex or from an existing SWIG
    implementation object. For clarity and documentation, prefer the explicit
    constructors:

    - :meth:`Point.from_cartesian`
    - :meth:`Sampler.point_in_face`
    - :meth:`Point.from_vertex`

    Attributes
    ----------
    position : np.ndarray
        3D position of the point.
    face : Face
        Face on which the point is located.
    """

    def __init__(self, point_impl: Union[_PointImpl, Vertex], *, mesh: "Mesh | None" = None):
        """Initialize a Point wrapper.

        Parameters
        ----------
        point_impl : _PointImpl or Vertex
            The underlying SWIG point object, or a Vertex to convert into a Point.
        mesh : Mesh, optional
            Required owner when wrapping a raw SWIG implementation. Vertex inputs
            already carry their owner.

        Raises
        ------
        TypeError
            If ``point_impl`` is not a supported type.
        """
        if isinstance(point_impl, Vertex):
            from . import _mesh_impl as _impl

            self._mesh = point_impl.mesh
            self._point_impl = _impl.point_from_vertex(self._mesh._mesh_impl, point_impl.id)
            return
        if isinstance(point_impl, _PointImpl):
            if mesh is None:
                raise TypeError("Wrapping a raw Point implementation requires its mesh owner.")
            self._mesh = mesh
            self._point_impl = point_impl
            return
        raise TypeError(
            f"Point expects a Vertex or _PointImpl. Got {type(point_impl)!r}."
        )

    @classmethod
    def from_vertex(cls, vertex: Vertex) -> "Point":
        """Create a point from a mesh vertex.

        Parameters
        ----------
        vertex : Vertex
            Vertex used to construct the point.

        Returns
        -------
        Point
            A point located on the given vertex.
        """
        return cls(vertex)

    @property
    def position(self) -> NDArray[np.float64]:
        """Get the 3D position of the point.

        Returns
        -------
        np.ndarray
            3D position vector ``[x, y, z]``.
        """
        return np.asarray(_impl.point_position(self._mesh._mesh_impl, self._point_impl)).reshape(-1)

    @property
    def face(self) -> "Face":
        """Get the face on which the point is located.

        Returns
        -------
        Face
            The face containing this point.
        """
        from .face import Face

        return Face(self._mesh, _impl.point_face(self._mesh._mesh_impl, self._point_impl))

    def describe(self) -> str:
        """Get a string description of the point.

        Returns
        -------
        str
            Description of the point.
        """
        return _impl.point_describe(self._mesh._mesh_impl, self._point_impl)

    @staticmethod
    def from_cartesian(mesh: "Mesh", position: ArrayLike) -> "Point":
        """Create a point from cartesian coordinates.

        Parameters
        ----------
        mesh : Mesh
            Mesh to which the point belongs.
        position : array_like, shape (3,)
            Cartesian coordinates ``[x, y, z]``.

        Returns
        -------
        Point
            New point created from cartesian coordinates.
        """
        position_vec = as_vector(position, size=3, name="position")
        return Point(_impl.from_cartesian(mesh._mesh_impl, position_vec), mesh=mesh)
