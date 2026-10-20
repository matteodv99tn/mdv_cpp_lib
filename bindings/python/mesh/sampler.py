from __future__ import annotations

from . import _mesh_impl as _impl
from .face import Face
from .mesh import Mesh
from .point import Point
from .tangent_vector import TangentVector


class Sampler:
    """Geometrically uniform, seedable sampler retaining its mesh owner.

    Faces and surface points are area-weighted, edges length-weighted, and
    vertices weighted by barycentric dual area. Tangent directions are uniform.
    Weights snapshot the geometry: rebuild after topology edits or non-uniform
    scaling/shear. Uniform scaling preserves relative weights.

    Parameters
    ----------
    mesh : Mesh
        Mesh to sample.
    seed : int
        Explicit unsigned 64-bit seed.
    """

    def __init__(self, mesh: Mesh, seed: int):
        self._mesh = mesh
        self._sampler_impl = _impl.Sampler(mesh._mesh_impl, seed)

    def face(self) -> Face:
        """Return a sampled face."""
        return Face(self._mesh, self._sampler_impl.face())

    def point_on_vertex(self) -> Point:
        """Return a sampled vertex point."""
        return Point(self._sampler_impl.point_on_vertex(), mesh=self._mesh)

    def point_on_edge(self) -> Point:
        """Return a length-weighted edge point, with t in [1e-6, 1 - 1e-6]."""
        return Point(self._sampler_impl.point_on_edge(), mesh=self._mesh)

    def point_in_face(self) -> Point:
        """Return an area-uniform surface point with barycentrics >= 1e-6."""
        return Point(self._sampler_impl.point_in_face(), mesh=self._mesh)

    def unit_tangent(self, point: Point) -> TangentVector:
        """Return a unit tangent at a face-interior point on this mesh."""
        if point._mesh is not self._mesh:
            raise ValueError("Point must belong to the sampler's mesh")
        return TangentVector.from_impl(
            self._sampler_impl.unit_tangent(point._point_impl), mesh=self._mesh
        )

    def points_on_vertices(self, n: int) -> list[Point]:
        """Return n sampled vertex points."""
        return [Point(p, mesh=self._mesh) for p in self._sampler_impl.points_on_vertices(n)]

    def points_on_edges(self, n: int) -> list[Point]:
        """Return n sampled edge-interior points."""
        return [Point(p, mesh=self._mesh) for p in self._sampler_impl.points_on_edges(n)]

    def points_in_faces(self, n: int) -> list[Point]:
        """Return n sampled face-interior points."""
        return [Point(p, mesh=self._mesh) for p in self._sampler_impl.points_in_faces(n)]
