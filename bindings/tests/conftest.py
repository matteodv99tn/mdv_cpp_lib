from __future__ import annotations

import pytest

from mdv.mesh import Mesh, Point  # type: ignore[import-not-found]


@pytest.fixture(scope="session")
def cube_mesh() -> Mesh:
    return Mesh.cube_angle()


@pytest.fixture()
def cube_points(cube_mesh: Mesh) -> tuple[Point, Point]:
    p0 = Point.from_vertex(cube_mesh.vertex(0))
    p1 = Point.from_vertex(cube_mesh.vertex(1))
    return p0, p1
