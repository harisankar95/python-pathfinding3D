# import the necessary packages
import numpy as np
import open3d as o3d
from pathfinding3D.core.diagonal_movement import DiagonalMovement
from pathfinding3D.core.grid import Grid
from pathfinding3D.finder.a_star import AStarFinder

# load voxel grid
voxel_grid = np.load("sample_map.npy")

# define start and end points
agent = [21, 21, 21]
target = [5, 38, 33]

# create grid representation and start and end nodes
grid = Grid(matrix=voxel_grid.tolist())
start = grid.node_from_array(agent)
end = grid.node_from_array(target)

# initialize A* finder
finder = AStarFinder(diagonal_movement=DiagonalMovement.only_when_no_obstacle)
path, runs = finder.find_path(start, end, grid)
path_cost = end.g
print(f"path cost: {path_cost:.4f}, path length: {len(path)}, runs: {runs}")


# visualize path in open3d
# find the obstacles and represent in blue
x_pt, y_pt, z_pt = np.where(voxel_grid == 0)
xyz_pt = np.zeros((np.size(x_pt), 3))
xyz_pt[:, 0] = x_pt
xyz_pt[:, 1] = y_pt
xyz_pt[:, 2] = z_pt

colors = [[0, 0, z/np.amax(z_pt)] for z in z_pt]

# add the start point in red
xyz_pt = np.vstack((xyz_pt, agent))
colors.append([1.0, 0, 0])

# add the end point in green
xyz_pt = np.vstack((xyz_pt, target))
colors.append([0, 1.0, 0])

# add the path without the start and end points in grey
for pt in path[1:-1]:
    colors.append([0.7, 0.7, 0.7])
    xyz_pt = np.vstack((xyz_pt, pt))

# Pass xyz to Open3D.o3d.geometry.PointCloud and visualize
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz_pt)
pcd.colors = o3d.utility.Vector3dVector(colors)

voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=1.0)
axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=15.0, origin=np.array([-3.0, -3.0, -3.0]))
o3d.visualization.draw_geometries([axes, voxel_grid], window_name='Voxel Env', width=1024, height=768)