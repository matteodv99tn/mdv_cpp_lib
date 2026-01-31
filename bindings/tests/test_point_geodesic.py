from __future__ import annotations

import numpy as np

from mdv.mesh import Mesh, Point  # type: ignore[import-not-found]


def test_point_from_cartesian_roundtrip(cube_mesh: Mesh) -> None:
    pos = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    point = Point.from_cartesian(cube_mesh, pos)
    assert point.position.shape == (3,)


def test_point_random(cube_mesh: Mesh) -> None:
    point = Point.random(cube_mesh)
    assert point.position.shape == (3,)


def test_build_geodesic(cube_mesh: Mesh) -> None:
    p0 = Point.from_vertex(cube_mesh.vertex(0))
    p1 = Point.from_vertex(cube_mesh.vertex(1))

    geodesic = cube_mesh.build_geodesic(p0, p1)
    path = geodesic.to_list()

    assert isinstance(path, list)
    assert len(path) > 0
    assert path[0].shape == (3,)
    assert geodesic.length() > 0.0
