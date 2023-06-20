import open3d
import numpy as np

class Open3DVisualizer():
    _viz = None

    def __init__(self):
        self._viz = open3d.visualization.Visualizer()
        self._viz.create_window()

    def destroy_window(self):
        self._viz.destroy_window()

    def plot3d(self, np_array):
        pcd = self._numpy_to_pcd(np_array)
        open3d.visualization.draw_geometries([pcd])

    def _numpy_to_pcd(self, np_array):
        pcd = open3d.geometry.PointCloud()
        pcd.points = open3d.utility.Vector3dVector(np_array[:, 0:3])
        return pcd
