# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# from matplotlib.animation import FuncAnimation
# import transforms3d as tf3d

# class Frame:
#     def __init__(self, x, y, z, roll, pitch, yaw, label, scale=1.0, ref_frame=None):
#         self.x = x
#         self.y = y
#         self.z = z
#         self.roll = roll
#         self.pitch = pitch
#         self.yaw = yaw
#         self.label = label
#         self.scale = scale
#         self.ref_frame = ref_frame  # Reference frame for the current frame

#     def get_transformation_matrix(self):
#         if self.ref_frame is None:
#             return tf3d.euler.euler2mat(self.yaw, self.pitch, self.roll, 'szyx')
#         else:
#             ref_matrix = self.ref_frame.get_transformation_matrix()
#             return np.dot(tf3d.euler.euler2mat(self.yaw, self.pitch, self.roll, 'szyx'), ref_matrix)

#     def plot(self, ax):
#         # Rotation matrix using transforms3d library
#         rotation_matrix = self.get_transformation_matrix()

#         # Length of the axes
#         axis_length = 1.0 * self.scale  # Adjust the scale factor

#         # Local axes in the object coordinate frame
#         local_axes = np.array([
#             [axis_length, 0, 0],
#             [0, axis_length, 0],
#             [0, 0, axis_length]
#         ])

#         # Transform local axes to world axes
#         world_axes = np.dot(local_axes, rotation_matrix.T) + np.array([self.x, self.y, self.z])

#         # Plot the local axes using quiver
#         ax.quiver(self.x, self.y, self.z, *rotation_matrix @ local_axes[0], color='red', label='X-axis')
#         ax.quiver(self.x, self.y, self.z, *rotation_matrix @ local_axes[1], color='green', label='Y-axis')
#         ax.quiver(self.x, self.y, self.z, *rotation_matrix @ local_axes[2], color='blue', label='Z-axis')

#         # Add label at the origin
#         ax.text(self.x, self.y, self.z, f"{self.label} (Scale: {self.scale})", color='black', fontsize=10, ha='center', va='center')


# class CoordinateSystem:
#     def __init__(self):
#         self.frames = []

#     def add_frame(self, frame):
#         self.frames.append(frame)

#     def update_plot(self, num, ax, animate_frame, new_x, new_y, new_z, new_roll, new_pitch, new_yaw):
#         ax.clear()

#         for frame in self.frames:
#             if frame == animate_frame:
#                 # Interpolate between current and new values
#                 alpha = min(1, num / 30.0)  # Interpolation factor (adjust the denominator for animation duration)
#                 frame.x = (1 - alpha) * frame.x + alpha * new_x
#                 frame.y = (1 - alpha) * frame.y + alpha * new_y
#                 frame.z = (1 - alpha) * frame.z + alpha * new_z
#                 frame.roll = (1 - alpha) * frame.roll + alpha * new_roll
#                 frame.pitch = (1 - alpha) * frame.pitch + alpha * new_pitch
#                 frame.yaw = (1 - alpha) * frame.yaw + alpha * new_yaw
#             frame.plot(ax)

#         # Set axis labels
#         ax.set_xlabel('X-axis')
#         ax.set_ylabel('Y-axis')
#         ax.set_zlabel('Z-axis')

#         # Set aspect ratio manually
#         max_extent = max(max(np.max(frame.x), np.max(frame.y), np.max(frame.z)) for frame in self.frames)
#         if max_extent != 0:
#             ax.set_xlim([-max_extent, max_extent])
#             ax.set_ylim([-max_extent, max_extent])
#             ax.set_zlim([-max_extent, max_extent])

#         # Set aspect ratio explicitly
#         ax.set_aspect('equal')

#     def animate_frame(self, frame, new_x, new_y, new_z, new_roll, new_pitch, new_yaw):
#         # Create a single figure and 3D axes
#         fig = plt.figure()
#         ax = fig.add_subplot(111, projection='3d')

#         ani = FuncAnimation(fig, self.update_plot, fargs=(ax, frame, new_x, new_y, new_z, new_roll, new_pitch, new_yaw),
#                             frames=30, interval=50, blit=False)
#         plt.show()
#         ani.pause()

# # Example usage for a coordinate system with multiple frames
# cs = CoordinateSystem()

# frame1 = Frame(0, 0, 0, np.radians(45), np.radians(30), np.radians(60), label='Frame 1', scale=2.0)
# frame2 = Frame(2, 2, 2, np.radians(30), np.radians(60), np.radians(45), label='Frame 2', ref_frame=frame1, scale=1.5)
# frame3 = Frame(-1, -1, -1, np.radians(60), np.radians(45), np.radians(30), label='Frame 3', ref_frame=frame2, scale=0.8)

# cs.add_frame(frame1)
# cs.add_frame(frame2)
# cs.add_frame(frame3)

# # Specify the frame to animate and new values
# cs.animate_frame(frame3, new_x=3, new_y=3, new_z=3, new_roll=np.radians(45), new_pitch=np.radians(30), new_yaw=np.radians(60))

import cv2
import numpy as np

# Create a black image
image = np.zeros((500, 500, 3), dtype=np.uint8)

# Define the center and radius of the hexagon
center = (250, 250)
radius = 100

# Calculate the vertices of the hexagon
hexagon_points = []
for i in range(6):
    x = int(center[0] + radius * np.cos(i * 2 * np.pi / 6))
    y = int(center[1] + radius * np.sin(i * 2 * np.pi / 6))
    hexagon_points.append((x, y))

# Convert the list of points to a NumPy array
hexagon_points = np.array(hexagon_points, dtype=np.int32)

# Reshape the array to fit OpenCV's requirements
hexagon_points = hexagon_points.reshape((-1, 1, 2))

# Draw the hexagon on the image
cv2.polylines(image, [hexagon_points], isClosed=True, color=(255, 255, 255), thickness=2)

# Display the image
cv2.imshow("Hexagon", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
