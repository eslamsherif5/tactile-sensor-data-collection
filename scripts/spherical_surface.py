import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

# Define the parametric equations for the sphere
def parametric_sphere(theta, phi, r):
    x = r * np.sin(np.radians(phi)) * np.cos(np.radians(theta))
    y = r * np.sin(np.radians(phi)) * np.sin(np.radians(theta))
    z = r * np.cos(np.radians(phi))
    return x, y, z

# Create a grid of values for theta and phi
theta_arr = np.linspace(0, 360, 20)
phi_arr = np.linspace(0, 70, 8)

# Specify radii for the inner spheres
radii_arr = np.linspace(0.05, 0.15, 3)

# Create a meshgrid from theta and phi
theta_grid, radii_grid = np.meshgrid(theta_arr, radii_arr)

# Coordinates of point A (center of the spheres)
A = np.array([0, 0, 0])

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# List to store poses of tiny frames for each phi value
all_poses = []

# Function to calculate rotation matrix for each point
def get_rotation_matrix(point, target):
    if np.allclose(point, target):
        return np.eye(3)
    else:
        z_axis = target - point
        norm_z = np.linalg.norm(z_axis)
        if norm_z == 0:
            return np.eye(3)
        else:
            z_axis /= norm_z
            x_axis = np.array([0, 0, 1])  # Radial direction
            y_axis = np.cross(z_axis, x_axis)
            norm_y = np.linalg.norm(y_axis)
            if norm_y == 0:
                return np.eye(3)
            else:
                y_axis /= norm_y
                x_axis = np.cross(y_axis, z_axis)
                return np.column_stack((x_axis, y_axis, z_axis))

# Plot the concentric spheres
for phi in phi_arr:
    x, y, z = parametric_sphere(theta_grid, phi, radii_grid)
    ax.plot_surface(x, y, z, alpha=0.5, label=f'Radius = {phi}')
    ax.scatter(x, y, z, c='red', marker='o', s=1)

    # # List to store poses of tiny frames for the current phi value
    # poses_at_phi = []

    # # Add a tiny frame at every point pointing radially to A
    # frame_length = 0.2  # Adjust the frame size as needed
    # for xi, yi, zi, phi_val in zip(x.flatten(), y.flatten(), z.flatten(), phi.flatten()):
    #     rotation_matrix = get_rotation_matrix(np.array([xi, yi, zi]), A)
    #     rotated_frame = np.dot(rotation_matrix, np.array([[frame_length, 0, 0],
    #                                                       [0, frame_length, 0],
    #                                                       [0, 0, frame_length]]).T)

    #     # Append the pose to the list
    #     poses_at_phi.append(rotation_matrix)

    #     # Quiver arrows directly on surface points
    #     ax.quiver(xi, yi, zi, rotated_frame[0, :], rotated_frame[1, :], rotated_frame[2, :],
    #               color='k', alpha=0.5, length=0.1, linewidth=1, arrow_length_ratio=0.5)

    # # Append the list of poses for the current phi to the overall list
    # all_poses.append(poses_at_phi)

# Set equal scaling for all axes
ax.set_box_aspect([np.ptp(coord) for coord in [x, y, z]])

# Set axis labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Show the plot
np.set_printoptions(suppress=True)
# Print the poses for each phi value
ax.scatter(0, 0, 0, c='red', marker='*', s=30)

# Print the list of poses for each phi value
for phi_idx, poses_at_phi in enumerate(all_poses):
    print(f'Phi = {phi[0, phi_idx]:.2f}:')
    for pose in poses_at_phi:
        print(pose)

plt.show()
