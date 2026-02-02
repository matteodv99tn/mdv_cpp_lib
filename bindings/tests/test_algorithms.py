from __future__ import annotations

import numpy as np

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
