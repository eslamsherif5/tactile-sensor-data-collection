import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import transforms3d as tf3d
from scipy.spatial.transform import Rotation as sp_rot

class Frame:
    def __init__(self, x, y, z, roll, pitch, yaw, label, scale=1, ref_frame=None):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.label = label
        self.scale = scale
        self.ref_frame = ref_frame  # Reference frame for the current frame

    def get_transformation_matrix(self):
        if self.ref_frame is None:
            # inertial frame
            transformation_matrix = np.zeros([4,4])
            transformation_matrix[3,3] = 1
            transformation_matrix[:3,:3] = sp_rot.from_euler('xyz',np.array([self.roll, self.pitch, self.yaw])).as_matrix()
            transformation_matrix[:3,3] = np.array([self.x, self.y, self.z])
            return transformation_matrix
        else:
            # non-inertial frame
            ref_matrix = self.ref_frame.get_transformation_matrix()
            transformation_matrix = np.zeros([4,4])
            transformation_matrix[3,3] = 1
            transformation_matrix[:3,:3] = ref_matrix[:3,:3] @ sp_rot.from_euler('xyz',np.array([self.roll, self.pitch, self.yaw])).as_matrix()
            transformation_matrix[:3,3] = np.add(ref_matrix[:3,:3] @ np.array([self.x, self.y, self.z]).T, ref_matrix[:3,3])
            return transformation_matrix

    def plot(self, ax: plt.Axes):
        # Rotation matrix using transforms3d library
        rotation_matrix = self.get_transformation_matrix()[:3,:3]
        translation_vec = self.get_transformation_matrix()[:3,3]

        # Length of the axes
        axis_length = 1.0 * self.scale

        # Local axes in the object coordinate frame
        local_axes = np.array([
            [axis_length, 0, 0],
            [0, axis_length, 0],
            [0, 0, axis_length]
        ])

        # Plot the local axes using quiver
        ax.quiver(*translation_vec, *rotation_matrix @ local_axes[0], color='red', label='X-axis')
        ax.quiver(*translation_vec, *rotation_matrix @ local_axes[1], color='green', label='Y-axis')
        ax.quiver(*translation_vec, *rotation_matrix @ local_axes[2], color='blue', label='Z-axis')

        # Add label at the origin
        ax.text(*translation_vec, self.label, color='black', fontsize=10, ha='center', va='center')
    
    def plot_arrow_to_parent(self, ax):
        if self.ref_frame is None:
            return
        
        # Plot an arrow from the parent frame to the current frame
        # Calculate the arrow direction as components (U, V, W)
        ref_frame_xyz = self.ref_frame.get_transformation_matrix()[:3,3].tolist()
        this_frame_xyz = self.get_transformation_matrix()[:3,3].tolist()
        arrow_direction = (np.array([ref_frame_xyz]) - np.array([this_frame_xyz])).tolist()[0]

        # Plot an arrow from the parent frame to the current frame
        print("this_frame_xyz")
        print(*this_frame_xyz)
        print("arrow_direction")
        print(arrow_direction)
        ax.quiver(this_frame_xyz[0], this_frame_xyz[1], this_frame_xyz[2],
                  arrow_direction[0], arrow_direction[1], arrow_direction[2],
                  color='gray', linestyle='dashed', arrow_length_ratio=0.1, linewidths=1)




class CoordinateSystem:
    def __init__(self):
        self.frames = []

    def add_frame(self, frame):
        self.frames.append(frame)

    def plot(self):
        # Create a single figure and 3D axes
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        for frame in self.frames:
            frame.plot(ax)
            frame.plot_arrow_to_parent(ax)

        # Set axis labels
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')

        # Set aspect ratio manually
        max_extent = max(max(max(np.max(frame.x), np.max(frame.y), np.max(frame.z)) for frame in self.frames), 4)
        if max_extent != 0:
            ax.set_xlim([-max_extent, max_extent])
            ax.set_ylim([-max_extent, max_extent])
            ax.set_zlim([-max_extent, max_extent])

        # Set aspect ratio explicitly
        ax.set_aspect('equal')

        # Show the plot
        plt.show()

# Example usage for a coordinate system with multiple frames
# cs = CoordinateSystem()

# frame1 = Frame(0, 0, 0, np.radians(45), np.radians(30), np.radians(60), label='Frame 1')
# frame2 = Frame(2, 2, 2, np.radians(30), np.radians(60), np.radians(45), label='Frame 2', ref_frame=frame1)
# frame3 = Frame(-1, -1, -1, np.radians(60), np.radians(45), np.radians(30), label='Frame 3', ref_frame=frame2)

# cs.add_frame(frame1)
# cs.add_frame(frame2)
# cs.add_frame(frame3)

# cs.plot()
