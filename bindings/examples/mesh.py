import os.path
import mdv.mesh
from mdv.mesh import Mesh, Point

mesh_dir = mdv.mesh.mesh_directory()
mesh_file = os.path.join(mesh_dir, "torus_simple.off")
mesh = Mesh.load_from_file(mesh_file)

p0 = Point.random(mesh)
p1 = Point.random(mesh)

print("Point p0:", p0.describe())
print("Point p1:", p1.describe())

geod = mesh.build_geodesic(p0, p1)
len = geod.length()
print("Geodesic path length:", len)
