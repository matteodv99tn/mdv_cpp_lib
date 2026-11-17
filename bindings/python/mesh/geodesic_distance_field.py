from __future__ import annotations

from collections.abc import Sequence
from typing import overload

import numpy as np
from numpy.typing import NDArray

from . import _mesh_impl as _impl
from .mesh import Mesh
from .point import Point
from .vertex import Vertex


class GeodesicDistanceField:
    """Minimum surface-geodesic distance to a fixed nonempty point set.

    Parameters
    ----------
    mesh : Mesh
        Borrowed surface, retained by this wrapper. Do not change its geometry
        or topology while using the field; reconstruct the field after changes.
    sources : Sequence[Point | Vertex]
        Source locations on this exact mesh. The sequence need not stay alive.

    Notes
    -----
    Precomputes one shortest-path tree per distinct source to avoid CGAL's
    multi-source visibility/boundary bugs. Query work and storage scale with the
    number of sources, but queries never rebuild trees or construct polylines.
    Unreachable queries return positive infinity. Calls on the same field must
    not run concurrently.
    Isolated vertices and wire edges are not surface locations and are rejected.
    """

    def __init__(self, mesh: Mesh, sources: Sequence[Point | Vertex]) -> None:
        self._mesh = mesh
        self._field_impl = _impl.GeodesicDistanceField(
            mesh._mesh_impl, self._points(sources)
        )

    def _point(self, point: Point | Vertex):
        if isinstance(point, Vertex):
            if point.mesh is not self._mesh:
                raise ValueError("Point must belong to the distance field's mesh")
            point = Point.from_vertex(point)
        if not isinstance(point, Point):
            raise TypeError("Expected a Point or Vertex")
        if point._mesh is not self._mesh:
            raise ValueError("Point must belong to the distance field's mesh")
        return point._point_impl

    def _points(self, points: Sequence[Point | Vertex]):
        return _impl.PointVector([self._point(point) for point in points])

    @overload
    def __call__(self, query: Point | Vertex) -> float: ...

    @overload
    def __call__(self, query: Sequence[Point | Vertex]) -> NDArray[np.float64]: ...

    def __call__(
        self, query: Point | Vertex | Sequence[Point | Vertex]
    ) -> float | NDArray[np.float64]:
        """Evaluate a point or an ordered batch of points on this mesh.

        Parameters
        ----------
        query : Point | Vertex | Sequence[Point | Vertex]
            Surface location or ordered batch of locations.

        Returns
        -------
        float or np.ndarray
            Nearest-source distance, or a float64 array of shape ``(N,)``.
            An empty query sequence returns an empty array.
        """
        if isinstance(query, (Point, Vertex)):
            return float(self._field_impl(self._point(query)))
        return np.asarray(
            self._field_impl.evaluate(self._points(query)), dtype=np.float64
        ).reshape(-1)
