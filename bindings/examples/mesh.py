import os.path
import mdv.mesh
from mdv.mesh import Mesh, Sampler

mesh_dir = mdv.mesh.mesh_directory()
mesh_file = os.path.join(mesh_dir, "torus_simple.off")
mesh = Mesh.load_from_file(mesh_file)

sampler = Sampler(mesh, 42)
p0 = sampler.point_in_face()
p1 = sampler.point_in_face()

print("Point p0:", p0.describe())
print("Point p1:", p1.describe())

geod = mesh.build_geodesic(p0, p1)
len = mdv.mesh.length(geod)
print("Geodesic path length:", len)
