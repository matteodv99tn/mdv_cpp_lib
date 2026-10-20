from __future__ import annotations

import numpy as np

from mdv.mesh import Mesh, Sampler  # type: ignore[import-not-found]


def test_mesh_matrices_shapes(cube_mesh: Mesh) -> None:
    vertices = cube_mesh.get_vertex_matrix()
    faces = cube_mesh.get_face_matrix()

    assert vertices.ndim == 2
    assert vertices.shape[1] == 3
    assert faces.ndim == 2
    assert faces.shape[1] == 3
    assert faces.dtype == np.int32

def test_vertex_and_face_vectors_are_1d(cube_mesh: Mesh) -> None:
    vertex = cube_mesh.vertex(0)
    face = cube_mesh.face(0)

    v_pos = vertex.position
    v_norm = vertex.normal
    f_norm = face.normal

    assert v_pos.shape == (3,)
    assert v_norm.shape == (3,)
    assert f_norm.shape == (3,)

    random_face = Sampler(cube_mesh, 42).face()
    assert random_face.id >= 0 and random_face.id < cube_mesh.num_faces


def test_vertex_handle_owns_mesh_and_preserves_queries() -> None:
    import gc
    import weakref

    from mdv.mesh import Point

    cube_mesh = Mesh.cube_angle()
    vertex = cube_mesh.vertex(0)
    assert vertex.mesh is cube_mesh
    assert vertex == cube_mesh.vertex(0)
    assert len({vertex, cube_mesh.vertex(0)}) == 1
    assert vertex != cube_mesh.vertex(1)
    assert vertex != Mesh.cube_angle().vertex(0)
    position = vertex.position.copy()
    assert cube_mesh.closest_vertex(position) == vertex
    assert "Vertex ID #0" in vertex.describe()
    assert np.isfinite(vertex.total_curvature())
    assert np.isfinite(vertex.gauss_curvature())
    owner = weakref.ref(cube_mesh)
    point = Point.from_vertex(vertex)
    del cube_mesh, vertex
    gc.collect()
    assert owner() is not None
    np.testing.assert_allclose(point.position, position)


def test_face_identity_and_owner_propagation() -> None:
    import gc
    import weakref

    from mdv.mesh import (
        FlatParameterisation,
        Point,
        TangentVector,
        exponential_map,
        logarithmic_map,
        parallel_transport,
    )

    mesh = Mesh.cube_angle()
    face = mesh.face(0)
    assert face.mesh is mesh
    assert face == mesh.face(0)
    assert len({face, mesh.face(0)}) == 1
    assert face != mesh.face(1)
    assert face != Mesh.cube_angle().face(0)
    assert "Face ID #0" in face.describe()
    from mdv.mesh import _mesh_impl

    assert _mesh_impl.vertices(mesh._mesh_impl, 0) == tuple(mesh.get_face_matrix()[0])
    assert len(_mesh_impl.neighbours(mesh._mesh_impl, 0)) == 3
    assert Sampler(mesh, 42).face().mesh is mesh
    p = Point.from_cartesian(mesh, [1.0, 0.4, 0.3])
    q = Point.from_cartesian(mesh, [1.0, 0.5, 0.3])
    assert p.face.mesh is mesh
    assert Sampler(mesh, 42).point_in_face().face.mesh is mesh
    assert Point.from_vertex(mesh.vertex(0)).face.mesh is mesh
    tv = TangentVector(p, [0.0, 0.01, 0.0])
    vectors = [
        tv,
        TangentVector.from_ambient_vector(p, [0.0, 0.01, 0.0]),
        TangentVector.from_tip_position(p, p.position + [0.0, 0.01, 0.0]),
        Sampler(mesh, 42).unit_tangent(p),
        tv.normalised(),
        logarithmic_map(p, q),
        parallel_transport(tv, q),
    ]
    for vector in vectors:
        assert vector.application_point().face.mesh is mesh
    assert exponential_map(tv).face.mesh is mesh
    parameterisation = FlatParameterisation(mesh)
    retrieved = parameterisation.retrieve(parameterisation.project(p))
    retained_face = retrieved.face
    assert retained_face.mesh is mesh
    expected_normal = retained_face.normal.copy()
    owner = weakref.ref(mesh)
    del mesh, face, p, q, tv, vectors, vector, parameterisation, retrieved
    gc.collect()
    assert owner() is retained_face.mesh
    np.testing.assert_allclose(retained_face.normal, expected_normal)


def test_lowlevel_halfedge_handles(cube_mesh: Mesh) -> None:
    from mdv.mesh import _mesh_impl

    mesh = cube_mesh._mesh_impl
    vertices = _mesh_impl.vertices(mesh, 0)
    edge = _mesh_impl.halfedge(mesh, vertices[0], 0)
    assert isinstance(edge, int)
    assert _mesh_impl.halfedge_face(mesh, edge) == 0
    assert _mesh_impl.twin(mesh, _mesh_impl.twin(mesh, edge)) == edge
    assert _mesh_impl.prev(mesh, _mesh_impl.prev(mesh, _mesh_impl.prev(mesh, edge))) == edge
    direction = np.asarray(_mesh_impl.direction(mesh, edge)).reshape(-1)
    inward = np.asarray(_mesh_impl.inbound_direction(mesh, edge)).reshape(-1)
    expected = cube_mesh.vertex(vertices[1]).position - cube_mesh.vertex(vertices[0]).position
    np.testing.assert_allclose(direction, expected)
    np.testing.assert_allclose(inward @ direction, 0.0, atol=1e-12)
    np.testing.assert_allclose(np.linalg.norm(inward), 1.0)
