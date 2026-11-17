from __future__ import annotations

import gc
from pathlib import Path
import weakref

import numpy as np
import pytest

import mdv.mesh as mesh  # type: ignore[import-not-found,attr-defined]

Point = mesh.Point  # type: ignore[attr-defined]
TangentVector = mesh.TangentVector  # type: ignore[attr-defined]
exponential_map = mesh.exponential_map  # type: ignore[attr-defined]
geodesic_resample = mesh.geodesic_resample  # type: ignore[attr-defined]
logarithmic_map = mesh.logarithmic_map  # type: ignore[attr-defined]
parallel_transport = mesh.parallel_transport  # type: ignore[attr-defined]
point_from_geodesic = mesh.point_from_geodesic  # type: ignore[attr-defined]


def test_geodesic_sampling(cube_mesh) -> None:
    vertices = cube_mesh.get_vertex_matrix()
    faces = cube_mesh.get_face_matrix()

    v0, v1, v2 = faces[0]
    p0_cart = 0.2 * vertices[v0] + 0.3 * vertices[v1] + 0.5 * vertices[v2]
    p1_cart = 0.3 * vertices[v0] + 0.3 * vertices[v1] + 0.4 * vertices[v2]

    p0 = Point.from_cartesian(cube_mesh, p0_cart)
    p1 = Point.from_cartesian(cube_mesh, p1_cart)
    geod = cube_mesh.build_geodesic(p0, p1)

    pt_mid = point_from_geodesic(geod, 0.5)
    assert pt_mid.shape == (3,)

    samples = geodesic_resample(geod, np.array([0.0, 0.5, 1.0], dtype=np.float64))
    assert samples.shape == (3, 3)


def test_parallel_transport_and_maps(cube_mesh) -> None:
    vertices = cube_mesh.get_vertex_matrix()
    faces = cube_mesh.get_face_matrix()

    v0, v1, v2 = faces[0]
    p0_cart = 0.2 * vertices[v0] + 0.3 * vertices[v1] + 0.5 * vertices[v2]
    p1_cart = 0.3 * vertices[v0] + 0.3 * vertices[v1] + 0.4 * vertices[v2]

    p0 = Point.from_cartesian(cube_mesh, p0_cart)
    p1 = Point.from_cartesian(cube_mesh, p1_cart)

    tv_log = logarithmic_map(p0, p1)
    assert tv_log.cartesian_vector().shape == (3,)

    tv_trans = parallel_transport(tv_log, p1)
    assert tv_trans.cartesian_vector().shape == (3,)

    p_exp = exponential_map(tv_log)
    assert p_exp.position.shape == (3,)


@pytest.fixture
def square_mesh():
    return mesh.Mesh.load_from_file(str(Path(mesh.mesh_directory()) / "square.off"))


def test_unit_tangent_polyline_and_geodesic(square_mesh) -> None:
    curve = np.array([[0.1, 0.1, 0], [0.4, 0.1, 0], [0.4, 0.1, 0], [0.4, 0.3, 0]])
    corner = np.linalg.norm(curve[1] - curve[0]) / sum(np.linalg.norm(np.diff(curve, axis=0), axis=1))
    tangent = mesh.unit_tangent(square_mesh, curve, corner)
    assert isinstance(tangent, TangentVector)
    np.testing.assert_allclose(tangent.application_point().position, curve[1])
    np.testing.assert_allclose(tangent.cartesian_vector(), [0, 1, 0], atol=1e-12)
    np.testing.assert_allclose(mesh.unit_tangent(square_mesh, curve.tolist(), 1).cartesian_vector(), [0, 1, 0], atol=1e-12)
    p = Point.from_cartesian(square_mesh, curve[0])
    q = Point.from_cartesian(square_mesh, curve[1])
    geodesic = square_mesh.build_geodesic(p, q)
    for s in (0, 0.5, 1):
        tangent = mesh.unit_tangent(square_mesh, geodesic, s)
        np.testing.assert_allclose(tangent.cartesian_vector(), [1, 0, 0], atol=1e-12)
        np.testing.assert_allclose(tangent.application_point().position, (1 - s) * curve[0] + s * curve[1])


def test_unit_tangent_continues_folded_endpoint(cube_mesh) -> None:
    curve = [[1, 1, 0.5], [1, 1, 1]]
    continued = mesh.unit_tangent(cube_mesh, curve, 1)
    np.testing.assert_allclose(np.linalg.norm(continued.cartesian_vector()), 1)
    np.testing.assert_allclose(continued.application_point().position, [1, 1, 1])
    # The cube vertex has cone angle 3*pi/2, not 2*pi.
    point = Point.from_cartesian(cube_mesh, [1, 1, 1])
    backward = TangentVector.from_ambient_vector(point, [0, 0, -1])
    expected = backward.rotated(3 * np.pi / 4)
    np.testing.assert_allclose(continued.cartesian_vector(), expected.cartesian_vector(), atol=1e-12)


def test_unit_tangent_invalid_inputs_and_boundaries(square_mesh) -> None:
    curve = [[0.1, 0.1, 0], [0.2, 0.1, 0]]
    for s in (-0.1, 1.1, np.nan, np.inf):
        with pytest.raises(RuntimeError):
            mesh.unit_tangent(square_mesh, curve, s)
    for invalid in (np.empty((0, 3)), [[0, 0, 0]], [[0, 0, 0], [0, 0, 0]],
                    [[0, 0, 0], [np.nan, 0, 0]], [[0.1, 0.1, 1], [0.2, 0.1, 1]]):
        with pytest.raises(RuntimeError):
            mesh.unit_tangent(square_mesh, invalid, 0.5)
    with pytest.raises(ValueError):
        mesh.unit_tangent(square_mesh, [[1, 2], [3, 4]], 0.5)
    with pytest.raises(RuntimeError):
        mesh.unit_tangent(square_mesh, [[0.25, 0.25, 0], [0, 0.25, 0]], 1)
    for s in (0, 0.5, 1):
        along = mesh.unit_tangent(square_mesh, [[0.8, 0, 0], [0.1, 0, 0]], s)
        np.testing.assert_allclose(along.cartesian_vector(), [-1, 0, 0], atol=1e-12)


def test_unit_tangent_keeps_mesh_alive() -> None:
    owner_mesh = mesh.Mesh.load_from_file(str(Path(mesh.mesh_directory()) / "square.off"))
    owner = weakref.ref(owner_mesh)
    tangent = mesh.unit_tangent(owner_mesh, [[0.1, 0.1, 0], [0.2, 0.1, 0]], 0.5)
    del owner_mesh
    gc.collect()
    assert owner() is not None
    np.testing.assert_allclose(tangent.application_point().position, [0.15, 0.1, 0])


def test_unit_tangent_near_crease():
    cube = mesh.Mesh.load_from_file(str(Path(mesh.mesh_directory()) / "cube.stl"))
    curve = [[50, -20, 40], [50, -20, 50], [40, -20, 50]]
    np.testing.assert_allclose(mesh.unit_tangent(cube, curve, 0.5 - 1e-12).cartesian_vector(),
                               [-1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(mesh.unit_tangent(cube, curve[:2], 1 - 1e-11).cartesian_vector(),
                               [-1, 0, 0], atol=1e-12)


def test_unit_tangent_segment_tolerance(square_mesh):
    square_mesh.scale(100)
    curve = [[10, 10, 0], [50 + 1e-8, 50 + 1e-8, 0]]
    for points, s in ((curve, 1), (curve[::-1], 0)):
        with pytest.raises(RuntimeError, match="segment must lie in one mesh triangle"):
            mesh.unit_tangent(square_mesh, points, s)


def test_unit_tangent_shallow_crease_and_near_vertex():
    cube = mesh.Mesh.load_from_file(str(Path(mesh.mesh_directory()) / "cube.stl"))
    for curve, remaining in (([[50, -20, 49], [50, 30, 50]], 2e-8),
                              ([[50, -20, 49], [50, 30, 50]], 4e-8),
                              ([[50, -45, 40], [50, -50, 50 - 1e-8]], 1e-8)):
        distance = np.linalg.norm(np.diff(curve, axis=0))
        expected = mesh.unit_tangent(cube, curve, 1).cartesian_vector()
        actual = mesh.unit_tangent(cube, curve, 1 - remaining / distance)
        np.testing.assert_allclose(actual.cartesian_vector(), expected, atol=1e-12)


def test_unit_tangent_does_not_skip_invalid_start(square_mesh):
    curve = [[0, 0, 0], [0.8, 0.8 * np.tan(1e-7), 0]]
    with pytest.raises(RuntimeError):
        mesh.unit_tangent(square_mesh, curve, 0)
