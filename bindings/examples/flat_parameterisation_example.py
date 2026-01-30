import os.path
import mdv.mesh
from mdv.mesh import FlatParameterisation, Mesh, Point

VISUALISE_MESH = True

# Test extract_normal_bounded_surface
mesh_dir = mdv.mesh.mesh_directory()
mesh_file = os.path.join(mesh_dir, "bunny.off")
full_mesh = Mesh.load_from_file(mesh_file)

print("Original (full) mesh:", full_mesh.name)
print("Original (full) mesh vertices:", full_mesh.num_vertices)
print("Original (full) mesh faces:", full_mesh.num_faces)

# Get the point at the tip of the bunnys nose
seed_point = Point(full_mesh.vertex(9793))
print("\nSeed point:", seed_point.describe())

# Extract a normal-bounded surface with a given angle limit
# This is necessary because only non-closed meshes can be parameterised!
max_angle = 50.0
print(
    f"\nExtracting normal-bounded surface with {max_angle} degree angle limit..."
)
mesh = Mesh.extract_normal_bounded_surface(full_mesh,
                                              seed_point,
                                              max_normal_angle=max_angle)

print("Normal-bounded mesh:", mesh.name)
print("Normal-bounded mesh vertices:", mesh.num_vertices)
print("Normal-bounded mesh faces:", mesh.num_faces)

if VISUALISE_MESH:
    import pyvista

    plotter = pyvista.Plotter()
    plotter.add_mesh(full_mesh.to_pyvista(), color="red")
    plotter.add_mesh(mesh.to_pyvista(), color="green")
    plotter.show()

print("\nCreating flat parameterisation of the normal-bounded mesh")
map = FlatParameterisation(mesh)
print("Is one-to-one mapping:", map.is_one_to_one_mapping())
min_uv = map.min_uv()
max_uv = map.max_uv()
print("    UV map min. coords: ", min_uv)
print("    UV map max. coords: ", max_uv)

print("\nAre min UV coordinates mapping to a point on the mesh?", map.is_inside_mesh(min_uv))
print("Are max UV coordinates mapping to a point on the mesh?", map.is_inside_mesh(max_uv))

pt = Point.random(mesh)
uv = map.project(pt)
print("\nRandom point on mesh:", pt.describe())
print("UV coordinate of the point:", uv)
print("Are UV coordinates mapping to a point on the mesh?", map.is_inside_mesh(uv))
print("Reconstructed point:", map.retrieve(uv).describe())

