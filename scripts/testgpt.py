import numpy as np
import transforms3d as tf3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def spherical_to_cartesian(radius, theta, phi):
    x = radius * np.sin(phi) * np.cos(theta)
    y = radius * np.sin(phi) * np.sin(theta)
    z = radius * np.cos(phi)
    return x, y, z

def generate_poses_between_spheres(radius_of_larger_sphere, max_theta, max_phi, radius_of_smaller_sphere, resolution):
    poses = []

    # Iterate through different theta and phi angles with a specified resolution
    for theta in np.linspace(0, 2 * np.pi, resolution):
        for phi in np.linspace(0, np.pi / 2, resolution):
            # Radial length based on the phi angle and the radius of the larger sphere
            radial_length = radius_of_larger_sphere * np.sin(phi)

            # Check if the pose is inside the limiting sphere
            if radial_length >= radius_of_smaller_sphere:
                # Convert spherical coordinates to Cartesian coordinates
                x, y, z = spherical_to_cartesian(radial_length, theta, phi)

                # Transformation matrix for B's pose relative to A
                pose_matrix = np.array([
                    [np.cos(theta), -np.sin(theta), 0, x],
                    [np.sin(theta), np.cos(theta), 0, y],
                    [0, 0, 1, z],
                    [0, 0, 0, 1]
                ])

                poses.append(pose_matrix)

    return poses

def plot_sphere(ax, radius, alpha=0.2, color='b'):
    # Plot a wireframe sphere
    u, v = np.mgrid[0:2*np.pi:100j, 0:np.pi/2:50j]
    x = radius * np.cos(u) * np.sin(v)
    y = radius * np.sin(u) * np.sin(v)
    z = radius * np.cos(v)
    ax.plot_wireframe(x, y, z, color=color, alpha=alpha)

# Parameters
radius_of_larger_sphere = 10.0
max_theta = np.radians(45)
max_phi = np.radians(45)
radius_of_smaller_sphere = 5.0
resolution = 50  # Adjust the resolution as needed

# Generate poses
poses = generate_poses_between_spheres(radius_of_larger_sphere, max_theta, max_phi, radius_of_smaller_sphere, resolution)

# Plotting
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot the larger sphere with transparency
plot_sphere(ax, radius_of_larger_sphere)

# Plot the smaller sphere centered at (0, 0, radius_of_larger_sphere)
plot_sphere(ax, radius_of_smaller_sphere, color='r', alpha=0.5)

# Plot the structured poses
for pose in poses:
    # Extract translation vector from the transformation matrix
    translation = pose[:3, 3]
    ax.scatter(*translation, c='r', marker='o')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Set equal aspect ratio for all axes
ax.set_box_aspect([np.ptp(axis.get_data_interval()) for axis in [ax.xaxis, ax.yaxis, ax.zaxis]])

plt.title('Larger Sphere with Structured 6-DOF Poses and Smaller Sphere')
plt.show()
