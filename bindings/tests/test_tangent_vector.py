from __future__ import annotations

import numpy as np

from mdv.mesh import Point, TangentVector  # type: ignore[import-not-found]


def test_tangent_vector_basic(cube_mesh) -> None:
    vertices = cube_mesh.get_vertex_matrix()
    faces = cube_mesh.get_face_matrix()

    v0, v1, v2 = faces[0]
    p0_cart = 0.2 * vertices[v0] + 0.3 * vertices[v1] + 0.5 * vertices[v2]
    point = Point.from_cartesian(cube_mesh, p0_cart)
    vec = np.array([0.1, 0.0, 0.0], dtype=np.float64)

    tv = TangentVector.from_ambient_vector(point, vec)
    assert tv.cartesian_vector().shape == (3,)

    tv_scaled = tv.normalised()
    assert tv_scaled.cartesian_vector().shape == (3,)
    assert isinstance(tv.type(), int)
