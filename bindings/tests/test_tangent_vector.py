from __future__ import annotations

import gc
import weakref

import numpy as np
import pytest

from mdv.mesh import Mesh, Point, TangentVector  # type: ignore[import-not-found]


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


def test_intrinsic_rotation(cube_mesh) -> None:
    vertices = cube_mesh.get_vertex_matrix()
    face = cube_mesh.get_face_matrix()[0]
    a, b, c = vertices[face]
    normal = np.cross(b - a, c - a)
    normal /= np.linalg.norm(normal)
    point = Point.from_cartesian(cube_mesh, 0.2 * a + 0.3 * b + 0.5 * c)
    vector = b - a
    tangent = TangentVector(point, vector)
    rotated = tangent.rotated(np.pi / 2)
    assert isinstance(rotated, TangentVector)
    np.testing.assert_allclose(rotated.cartesian_vector(), np.cross(normal, vector), atol=1e-10)
    np.testing.assert_array_equal(tangent.cartesian_vector(), vector)
    assert rotated._mesh is cube_mesh
    assert rotated.application_point()._mesh is cube_mesh
    assert tangent.rotate(np.pi / 2) is None
    np.testing.assert_allclose(tangent.cartesian_vector(), rotated.cartesian_vector())
    np.testing.assert_allclose(tangent.rotated(-np.pi / 2).cartesian_vector(), vector, atol=1e-10)
    for angle in (np.nan, np.inf, -np.inf):
        with pytest.raises(RuntimeError, match="finite"):
            tangent.rotate(angle)
    np.testing.assert_allclose(tangent.cartesian_vector(), rotated.cartesian_vector())


def test_rotation_at_edge_and_vertex(cube_mesh) -> None:
    for position, vector, turn in (
        ([1, 1, 0.5], [0, 0, 2], 2 * np.pi),
        ([1, 1, 1], [0, 0, -2], 1.5 * np.pi),
    ):
        point = Point.from_cartesian(cube_mesh, position)
        tangent = TangentVector.from_ambient_vector(point, vector)
        np.testing.assert_allclose(tangent.rotated(turn).cartesian_vector(), vector, atol=1e-12)
        rotated = tangent.rotated(-0.37)
        np.testing.assert_allclose(rotated.application_point().position, position)
        np.testing.assert_allclose(np.linalg.norm(rotated.cartesian_vector()), 2)
        np.testing.assert_allclose(rotated.rotated(0.37).cartesian_vector(), vector, atol=1e-12)
        tangent.scale(0)
        np.testing.assert_array_equal(tangent.rotated(1).cartesian_vector(), np.zeros(3))


def test_rotation_rejects_boundary(cube_mesh) -> None:
    point = Point.from_cartesian(cube_mesh, [1, 0.5, 0])
    tangent = TangentVector(point, [0, 0, 1])
    original = tangent.cartesian_vector().copy()
    with pytest.raises(RuntimeError, match="boundary"):
        tangent.rotate(0.2)
    np.testing.assert_array_equal(tangent.cartesian_vector(), original)
    assert tangent.rotate(0) is None


def test_rotated_copy_keeps_mesh_alive() -> None:
    mesh = Mesh.cube_angle()
    owner = weakref.ref(mesh)
    point = Point.from_cartesian(mesh, [1, 1, 0.5])
    tangent = TangentVector.from_ambient_vector(point, [0, 0, 2])
    rotated = tangent.rotated(0.37)
    del mesh, point, tangent
    gc.collect()
    assert owner() is not None
    np.testing.assert_allclose(rotated.application_point().position, [1, 1, 0.5])
    assert np.isfinite(rotated.tip()).all()


def test_rotation_near_full_turn(cube_mesh) -> None:
    point = Point.from_cartesian(cube_mesh, [1, 1, 0.5])
    tangent = TangentVector.from_ambient_vector(point, [0, 0, 2])
    for angle in (-1.0004e-12, -1e-12, -9.998e-13):
        rotated = tangent.rotated(angle)
        np.testing.assert_allclose(rotated.cartesian_vector(), [0, 0, 2], atol=1e-10)
