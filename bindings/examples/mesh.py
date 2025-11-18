import os.path
import mdv
from mdv.mesh import Mesh, Point

mesh = mdv.mesh.load_from_file(
    os.path.join(mdv.mesh.mesh_directory(), "torus_simple.off"))

p0 = mdv.mesh.Point.random(mesh)
p1 = mdv.mesh.Point.random(mesh)

print("Point p0:", p0.describe())
print("Point p1:", p1.describe())

print("Geodesic path length:", mdv.mesh.length(mesh.build_geodesic(p0, p1)))
