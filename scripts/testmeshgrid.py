import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Function to generate points on a sphere
def parametric_sphere(theta, phi, r):
    x = r * np.sin(np.radians(phi)) * np.cos(np.radians(theta))
    y = r * np.sin(np.radians(phi)) * np.sin(np.radians(theta))
    z = r * np.cos(np.radians(phi))
    return x, y, z

# Sample data: lists of thetas and radii for concentric spheres
theta_values = np.linspace(0, 360, 4)
radii_values = np.linspace(1, 3, 3)

# Plotting concentric spheres
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

poses = []
phi_arr = np.linspace(0, 30, 3)
for phi in phi_arr:
    x_list = y_list = z_list = np.array([])
    for r in radii_values:
        for theta in theta_values:
            # Fix phi to create circles on each sphere
            x, y, z = parametric_sphere(theta, phi, r)
            x_list = np.append(x_list, x)
            y_list = np.append(y_list, y)
            z_list = np.append(z_list, z)

    # Reshape x, y, z to be 2D arrays
    x_list = x_list.reshape(len(theta_values)*len(radii_values), 1)
    y_list = y_list.reshape(len(theta_values)*len(radii_values), 1)
    z_list = z_list.reshape(len(theta_values)*len(radii_values), 1)
    
    xyz = np.hstack([x_list, y_list, z_list]).reshape(-1,3)
    
    print(xyz.shape)
    
    # poses.append(list(zip(x_list.tolist(), y_list.tolist(), z_list.tolist())))
    poses.append(xyz)
    
    # Scatter plot points
    ax.scatter(x_list, y_list, z_list, color='b', alpha=0.6)

    # Surface plot
    ax.plot_surface(x_list, y_list, z_list, alpha=0.5)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Concentric Spheres with Varying Radii and Thetas')
print(len(poses))
print(len(poses[0]))
for pose in poses:
    for i in pose:
        print(i)
    print()
    
plt.show()
