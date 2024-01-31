#!/usr/bin/python3

import json
import random
import time

from matplotlib import pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import Pose, Vector3, PoseStamped, Twist, Quaternion, Point
from std_msgs.msg import Float64, Bool
from utils import *
from plot_3d_axes import CoordinateSystem, Frame
from scipy.spatial.transform import Rotation as sp_rot
from tactile_data_collection.srv import moveRobot,desiredTCP,moveRobotRelative
from kinematics import RobotKinematics

class DataGenerator:
    
    def json_file_to_dict(self, calibration_config_file: str) -> dict:
        if not calibration_config_file.endswith('.json'):
            calibration_config_file += '.json'
        try:
            with open(calibration_config_file) as inputfile:
                return json.load(inputfile)
        except Exception as e:
            print(e)
            print(RED + "Error: Config file '" + calibration_config_file + "' not found." + RST)
            exit()
            
    def __init__(self, camera_calibration_results, tact_calibration) -> None:
        
        # load extrinsic calibration data
        ext_camera_cal = self.json_file_to_dict(camera_calibration_results)
        ext_tactile_cal = self.json_file_to_dict(tact_calibration)
        
        # Transformation matrix of the calibration target (i.e. starting point) relative to the robot base
        # TODO: make the data collection independent of the location of the calibration target
        self.base_T_target_t = np.array(ext_camera_cal['base_T_target']['translation_vec'])
        self.base_T_target_rot = np.array(ext_camera_cal['base_T_target']['rotation_mat'])
        # self.target_correction = sp_rot.from_euler('zyx', [90, 0, 180], degrees=True).as_matrix()
        # self.base_T_target_rot = self.target_correction @ self.base_T_target_rot

        self.cam_T_tact_t = np.array(ext_tactile_cal['translation_vec'])
        self.cam_T_tact_rot = sp_rot.from_euler('xyz', ext_tactile_cal['euler_angles']).as_matrix()
        
        # defining frames and coordinate system
        world = Frame(0, 0, 0, np.radians(0), np.radians(0), np.radians(0), label='world', scale=0.5)
        ur_base = Frame(0, 1, 0, np.radians(0), np.radians(0), np.radians(90), label='ur_base', scale=0.25, ref_frame=world)
        tcp = Frame(0, -0.75, 0.75, np.radians(180), np.radians(0), np.radians(180), label='tcp', scale=0.25, ref_frame=ur_base)
        camera = Frame(*ext_camera_cal['tcp_T_cam']['translation_vec'], *ext_camera_cal['tcp_T_cam']['euler_angles_rad'], label='camera', scale=0.25, ref_frame=tcp)
        tactile = Frame(*ext_tactile_cal['translation_vec'], *ext_tactile_cal['euler_angles'], label='tactile', scale=0.25, ref_frame=camera)
        charuco_board = Frame(*self.base_T_target_t, *sp_rot.from_matrix(self.base_T_target_rot).as_euler('xyz'), label='charuco', scale=0.25, ref_frame=ur_base)
        
        cs = CoordinateSystem()
        cs.add_frame(world)
        cs.add_frame(ur_base)
        cs.add_frame(tcp)
        cs.add_frame(camera)
        cs.add_frame(tactile)
        cs.add_frame(charuco_board)
        
        # cs.plot()
        # plt.show()

        self.k = RobotKinematics()
        # self.k.set_transform('ur_base', 'charuco', 
        #                      np.vstack([np.hstack([np.eye(3),
        #                                            self.base_T_target_t.reshape(3,1)]),
        #                                 np.array([0,0,0,1])]),
        #                      mode='static')
        
        self.stop = False

        # ros stuff
        # TODO: remove the print statements
        self.contact_status_publisher = rospy.Publisher("contact_status", Bool, queue_size=10)
        self.contact_angle_deg_publisher = rospy.Publisher("contact_angle_deg", Vector3, queue_size=2)
        self.sensor_depth_publisher = rospy.Publisher("sensor_depth", Float64, queue_size=10)
        self.cmd_pose_publisher = rospy.Publisher("/ur_cmd_pose", Pose, queue_size=2)
        self.cmd_vel_publisher = rospy.Publisher("/ur_cmd_vel", Twist, queue_size=2)
        # self.pose_subs = rospy.Subscriber("/dvs/pose", PoseStamped, self.dvs_pose_callback, queue_size=2)
        self.rate = rospy.Rate(100)
        
        self.moveRobotCall = rospy.ServiceProxy('move_ur', moveRobot)  
        self.adjustTCPCall = rospy.ServiceProxy('move_TCP', moveRobot)
        self.relativeTCPCall = rospy.ServiceProxy('move_ur_relative', moveRobotRelative)

        # Define in plane frame
        self.robot_pose = PoseStamped()
        self.org_pose = Pose()
        self.org_pose.position.x = 0
        self.org_pose.position.y = 0

        self.cmd_pose = Pose()
        self.cmd_pose.position.x = 0
        self.cmd_pose.position.y = 0

        self.cmd_vel = Twist()

        self.contact_status = Bool()
        self.contact_angle = Vector3()
        self.sensor_depth = Float64()

        # specify tactile tip angles for data collection
        self.max_phi = 20 #  degrees
        self.n_phi = 5
        self.phi_list = [np.radians(5)] #np.linspace(0, np.radians(self.max_phi), self.n_phi).tolist()

        # specify number of rotation divisions
        self.n_divisions = 360//90 + 1
        self.n_samples_per_division = 1
        
        # specify z values for contact (in target frame)
        self.z_at_contact_thresh = 0.07
        self.z_at_max_depth = 0.03
        self.z_at_min_depth = 0.06
        
        # data collection pose list
        self.pose_msgs = []
        self.theta = np.linspace(0, 360, 5)
        self.phi = np.linspace(0, 20, 3)
        self.theta, self.phi = np.meshgrid(self.theta, self.phi)
        self.radii = np.linspace(0.15, 0.05, 5)
        self.A = np.array([0, 0, 0])
        self.frames = []
        
    def parametric_sphere(self, theta, phi, r):
        x = r * np.sin(np.radians(phi)) * np.cos(np.radians(theta))
        y = r * np.sin(np.radians(phi)) * np.sin(np.radians(theta))
        z = r * np.cos(np.radians(phi))
        return x, y, z
        
    def get_rotation_matrix(self, point, target):
        z_axis = target - point
        z_axis /= np.linalg.norm(z_axis)
        x_axis = np.array([0, 1, 0])  # Radial direction
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, z_axis)
        return np.column_stack((x_axis, y_axis, z_axis))

    def collect_data(self):
        t = np.array([0.,0,0])
        q = sp_rot.from_euler('xyz', [0, 0, 0], degrees=True).as_quat()
        target_Pose_tactile = Pose()    # pose of tactile sensor tip w.r.t the charuco board (target)
        target_Pose_tactile.position = Point(x=t[0],
                                             y=t[1],
                                             z=t[2])
        target_Pose_tactile.orientation = Quaternion(x=q[0],
                                                     y=q[1],
                                                     z=q[2],
                                                     w=q[3])
        for f in self.frames:
            self.relativeTCPCall('tactile', f, target_Pose_tactile)
            
    def generate_poses(self):
        # Plot the concentric spheres
        j = 0
        for r in self.radii:
            x, y, z = self.parametric_sphere(self.theta, self.phi, r)
            # ax.plot_surface(x, y, z, alpha=0.5, label=f'Radius = {r}')
            # ax.scatter(x, y, z, c='red', marker='o', s=1)
            k =0
            # Add a tiny frame at every point pointing radially to A
            frame_length = 0.02  # Adjust the frame size as needed
            for xi, yi, zi in zip(x.flatten(), y.flatten(), z.flatten()):
                rotation_matrix = self.get_rotation_matrix(np.array([xi, yi, zi]), self.A)
                # print(GRN + f'{rotation_matrix}' + RST)
                self.k.set_transform('charuco', f'{j}{k}', 
                                     np.vstack([np.hstack([rotation_matrix,
                                                   np.array([xi,yi,zi]).reshape(3,1)]),
                                        np.array([0,0,0,1])]),
                             mode='static')
                self.frames.append(f'{j}{k}')
                k += 1
            j += 1
                # ax.quiver(xi, yi, zi, rotated_frame[0, :], rotated_frame[1, :], rotated_frame[2, :],
                #         color='k', alpha=0.5)
