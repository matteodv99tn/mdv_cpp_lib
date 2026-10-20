import gc
import weakref

import numpy as np
import pytest

from mdv.mesh import Mesh, Sampler


def test_reproducible_samples(cube_mesh):
    a, b, c = (Sampler(cube_mesh, seed) for seed in (42, 42, 43))
    faces = [a.face().id for _ in range(50)]
    assert faces == [b.face().id for _ in range(50)]
    assert faces != [c.face().id for _ in range(50)]
    for method in ("points_on_vertices", "points_on_edges", "points_in_faces"):
        pa = getattr(a, method)(50)
        pb = getattr(b, method)(50)
        pc = getattr(c, method)(50)
        assert isinstance(pa, list) and len(pa) == 50
        np.testing.assert_array_equal([p.position for p in pa], [p.position for p in pb])
        assert not np.array_equal([p.position for p in pa], [p.position for p in pc])
        assert getattr(a, method)(0) == []
    p = a.point_in_face()
    np.testing.assert_array_equal(p.position, b.point_in_face().position)
    np.testing.assert_array_equal(a.unit_tangent(p).cartesian_vector(), b.unit_tangent(p).cartesian_vector())


def test_sampler_and_results_keep_mesh_alive():
    mesh = Mesh.cube_angle()
    owner = weakref.ref(mesh)
    sampler = Sampler(mesh, 2**64 - 1)
    del mesh
    gc.collect()
    assert owner() is not None
    points = [sampler.point_on_vertex(), sampler.point_on_edge(), sampler.point_in_face()]
    face = sampler.face()
    tangent = sampler.unit_tangent(points[-1])
    del sampler
    gc.collect()
    assert face.mesh is owner()
    assert tangent.application_point().face.mesh is owner()
    for point in points:
        assert point.face.mesh is owner()
        assert np.isfinite(point.position).all()


def test_sampler_rejects_foreign_point(cube_mesh):
    sampler = Sampler(cube_mesh, 42)
    other = Sampler(Mesh.cube_angle(), 42).point_in_face()
    with pytest.raises(ValueError):
        sampler.unit_tangent(other)


def test_sampler_rejects_boundary_tangents(cube_mesh):
    sampler = Sampler(cube_mesh, 42)
    for point in (sampler.point_on_vertex(), sampler.point_on_edge()):
        with pytest.raises(RuntimeError):
            sampler.unit_tangent(point)


@pytest.mark.parametrize("seed", [-1, 2**64])
def test_seed_range(cube_mesh, seed):
    with pytest.raises(OverflowError):
        Sampler(cube_mesh, seed)
