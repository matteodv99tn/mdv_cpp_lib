import gc
import weakref

import numpy as np
import pytest

from mdv.mesh import GeodesicDistanceField, Mesh, Point, Sampler, Vertex


METHODS = ("points_on_vertices", "points_on_edges", "points_in_faces")


@pytest.mark.parametrize("source_method", METHODS)
@pytest.mark.parametrize("query_method", METHODS)
def test_pairwise_distances(cube_mesh, source_method, query_method):
    sampler = Sampler(cube_mesh, 42)
    sources = getattr(sampler, source_method)(20)
    queries = getattr(sampler, query_method)(10)
    field = GeodesicDistanceField(cube_mesh, sources)
    expected = [
        min(cube_mesh.build_geodesic(s, q).length() for s in sources)
        for q in queries
    ]
    batch = field(queries)
    assert batch.shape == (10,)
    assert batch.dtype == np.float64
    np.testing.assert_allclose(batch, expected, rtol=1e-9, atol=1e-9)
    np.testing.assert_array_equal(batch, [field(q) for q in queries])
    np.testing.assert_allclose(field(sources), 0.0, atol=1e-9)
    assert field([]).shape == (0,)


def test_vertices_duplicates_and_source_storage(cube_mesh):
    sources = [cube_mesh.vertex(0), cube_mesh.vertex(0), cube_mesh.vertex(1)]
    field = GeodesicDistanceField(cube_mesh, sources)
    sources.clear()
    assert field(cube_mesh.vertex(0)) == pytest.approx(0.0)
    query = Point.from_vertex(cube_mesh.vertex(2))
    assert field(query) == pytest.approx(min(
        cube_mesh.build_geodesic(Point.from_vertex(cube_mesh.vertex(i)), query).length()
        for i in (0, 1)
    ))


def test_field_retains_mesh():
    mesh = Mesh.cube_angle()
    owner = weakref.ref(mesh)
    field = GeodesicDistanceField(mesh, [mesh.vertex(0)])
    del mesh
    gc.collect()
    assert owner() is not None
    assert field(owner().vertex(0)) == pytest.approx(0.0)
    del field
    gc.collect()
    assert owner() is None


def test_invalid_inputs(cube_mesh):
    with pytest.raises(RuntimeError, match="nonempty"):
        GeodesicDistanceField(cube_mesh, [])
    foreign = Mesh.cube_angle()
    for point in (foreign.vertex(0), Point.from_vertex(foreign.vertex(0))):
        with pytest.raises(ValueError, match="mesh"):
            GeodesicDistanceField(cube_mesh, [point])
        field = GeodesicDistanceField(cube_mesh, [cube_mesh.vertex(0)])
        with pytest.raises(ValueError, match="mesh"):
            field(point)
        with pytest.raises(ValueError, match="mesh"):
            field([cube_mesh.vertex(0), point])
    with pytest.raises(TypeError, match="Point or Vertex"):
        GeodesicDistanceField(cube_mesh, [object()])
    with pytest.raises(TypeError, match="Point or Vertex"):
        field([object()])
    invalid = Vertex(cube_mesh, cube_mesh.num_vertices + 10)
    with pytest.raises(RuntimeError):
        GeodesicDistanceField(cube_mesh, [invalid])
    with pytest.raises(RuntimeError):
        field(invalid)


def test_disconnected_surface():
    mesh = Mesh(
        vertices=[[0, 0, 0], [1, 0, 0], [0, 1, 0],
                  [3, 0, 0], [4, 0, 0], [3, 1, 0]],
        faces=[[0, 1, 2], [3, 4, 5]],
    )
    field = GeodesicDistanceField(mesh, [mesh.vertex(0)])
    assert np.isposinf(field(mesh.vertex(3)))
    assert np.isposinf(field([mesh.vertex(3)]))[0]
