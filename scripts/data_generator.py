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
        self.k.set_transform('ur_base', 'charuco', 
                             np.vstack([np.hstack([self.base_T_target_rot,
                                                   self.base_T_target_t.reshape(3,1)]),
                                        np.array([0,0,0,1])]),
                             mode='static')
        
        self.stop = False

        # ros stuff
        # TODO: remove the print statements
        self.contact_status_publisher = rospy.Publisher("contact_status", Bool, queue_size=10)
        self.contact_angle_deg_publisher = rospy.Publisher("contact_angle_deg", Vector3, queue_size=2)
        self.sensor_depth_publisher = rospy.Publisher("sensor_depth", Float64, queue_size=10)
        self.cmd_pose_publisher = rospy.Publisher("/ur_cmd_pose", Pose, queue_size=2)
        self.cmd_vel_publisher = rospy.Publisher("/ur_cmd_vel", Twist, queue_size=2)
        self.pose_subs = rospy.Subscriber("/dvs/pose", PoseStamped, self.dvs_pose_callback, queue_size=2)
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
        
    def rotvec_to_quaternion_msg(self, rx, ry, rz):
        quat = sp_rot.from_rotvec([rx, ry, rz]).as_quat()
        quat_msg = Quaternion(x=quat[0],
                              y=quat[1],
                              z=quat[2],
                              w=quat[3])
        return quat_msg
    
    def target_relative_to_base(self, pose_msg: Pose):
        # base_P_tactile = base_P_target + base_Rot_target * target_P_tactile
        msg_position = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        msg_quaternion = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
        
        rel_position = self.base_T_target_t.reshape(3,-1) + \
            self.base_T_target_rot @ np.array([*msg_position]).reshape(3,-1)
        rel_orientation = self.base_T_target_rot @ sp_rot.from_quat([*msg_quaternion]).as_matrix()
        rel_orientation = sp_rot.from_matrix(rel_orientation).as_quat()
        
        return Pose(position=Point(x=rel_position[0], 
                                   y=rel_position[1], 
                                   z=rel_position[2]),
                    orientation=Quaternion(x=rel_orientation[0],
                                           y=rel_orientation[1],
                                           z=rel_orientation[2],
                                           w=rel_orientation[3]))
        
    def frame_relative_to_base(self, frame: str):
        self.k.receive_transform(parent='ur_base', frame=str(frame))
                
    def pose_relative_to_target(self, pose_msg: Pose):
        # base_P_x = base_P_target + base_Rot_target * target_P_x
        msg_position = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        msg_quaternion = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
        
        rel_position = -self.base_T_target_t.reshape(3,-1) + \
            np.array([*msg_position]).reshape(3,-1) + \
                self.base_T_target_rot.T @ np.array([*msg_position]).reshape(3,-1)

        rel_orientation = self.base_T_target_rot.T @ sp_rot.from_quat([*msg_quaternion]).as_matrix()
        rel_orientation = sp_rot.from_matrix(rel_orientation).as_quat()
        
        return Pose(position=Point(x=rel_position[0], 
                                   y=rel_position[1], 
                                   z=rel_position[2]),
                    orientation=Quaternion(x=rel_orientation[0],
                                           y=rel_orientation[1],
                                           z=rel_orientation[2],
                                           w=rel_orientation[3]))

    def dvs_pose_callback(self, pose_msg):
        pass
            
            
    # def move_tactile_tip_to_target_with_offset(self, offset):
    #     starting_cmd_pose = Pose()
    #     starting_cmd_pose.position = Point(x=0., y=0., z=offset)  # TODO: change z to -z_at_min_depth
    #     quat = sp_rot.from_matrix(self.base_T_target_rot).as_quat()
    #     starting_cmd_pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    #     self.cmd_pose_publisher.publish(self.target_relative_to_base(starting_cmd_pose))

    def move_tactile_tip_to_target_with_z_offset(self, offset):
        # if not -0.0005 > -0.001:
        if offset < self.z_at_max_depth: # fool-safe check
            print(f'{RED}The requested tactile tip Z offset is below the specified minimum Z. Exiting.{RST}')
            return
        
        t = np.array([0.,0,offset])
        q = sp_rot.from_euler('xyz', [180, 0, 90], degrees=True).as_quat()
        target_Pose_tactile = Pose()    # pose of tactile sensor tip w.r.t the charuco board (target)
        target_Pose_tactile.position = Point(x=t[0],
                                             y=t[1],
                                             z=t[2])
        target_Pose_tactile.orientation = Quaternion(x=q[0],
                                                     y=q[1],
                                                     z=q[2],
                                                     w=q[3])
        # self.relativeTCPCall('child', 'parent', target_Pose_tactile)
        self.relativeTCPCall('tactile', 'charuco', target_Pose_tactile)
        
    def collect_data(self):
        '''
        STEPS
        -----
        1. Generate a list of <n_divisions> poses
           (e.g. n_divisions = 360/15+1 = 25 poses from 0 to 360 degs)
        2. Move the tactile tip to the same pose of the target with 
           an upward offset in z (-ve value) since the target orientation 
           is such that z is pointing downwards
        3. Go to pose_i in poses list
        4. For each pose_i, we go to -z_at_min_depth and then go to a random z position
           between -z_at_max_depth and -z_at_contact_threshold
           This is repeated for <n_samples_per_division> times
        6.
        '''
        # Adjust the tactile tip to the same pose of the target with an upward offset in z (-ve value)
        self.generate_data_collection_poses()
        for pose_msg in self.pose_msgs:
            
            # move sensor tip to be barely touching the target
            # self.move_tactile_tip_to_target_with_z_offset(self.z_at_contact_thresh)
            
            # check for contact
            target_P_dvs = self.pose_relative_to_target(pose_msg)
            if (-target_P_dvs.position.z < self.z_at_contact_thresh):
                self.contact_status.data = True
                self.sensor_depth.data = target_P_dvs.position.z + self.z_at_contact_thresh
            else:
                self.contact_status.data = False
                self.sensor_depth.data = 0.
            
            # move to pose
            print(f'{pose_msg}')
            if pose_msg.position.z < self.z_at_max_depth:
                print(f'{RED}The requested tactile tip Z offset is below the specified maximum depth.{RST}')
                continue
            self.moveRobotCall('tactile', pose_msg)
            time.sleep(0.25)
        
    def generate_data_collection_poses(self):
        for i in range(self.n_divisions):
            if i == 0:
                # sensor is normal to the charuco board
                cmd_pose = Pose()
                cmd_pose.orientation = self.rotvec_to_quaternion_msg(*sp_rot.from_euler('xyz', [180,0,90], degrees=True).as_rotvec())
                for j in range(self.n_samples_per_division):
                    cmd_pose.position = Point(x=0., y=0., z=random.uniform(self.z_at_max_depth, self.z_at_contact_thresh))
                    # print(cmd_pose)
                    self.pose_msgs.append(self.target_relative_to_base(cmd_pose))
                    print(f'{BLU}Sample: {i},{j}{RST}')
                    print(f'\t{GRN}Z: {cmd_pose.position.z}{RST}')
                    print(f'\t{GRN}Theta: 0.0{RST}')
                    print(f'\t{GRN}Phi: 0.0{RST}')
            else:
                theta = i * 2 * np.pi / (self.n_divisions - 1)
                for phi in self.phi_list:
                    rx = phi * np.cos(theta)
                    ry = phi * np.sin(theta)
                    rz = 0.
                    cmd_pose = Pose()
                    cmd_pose.orientation = self.rotvec_to_quaternion_msg(rx, ry, rz)

                    for j in range(self.n_samples_per_division):
                        z = (random.uniform(self.z_at_max_depth, self.z_at_contact_thresh))
                        x = self.org_pose.position.x + np.tan(phi) * np.sin(theta) * (z - self.z_at_min_depth)
                        y = self.org_pose.position.y + np.tan(phi) * np.cos(theta) * (z - self.z_at_min_depth)
                        cmd_pose.position = Point(x=x, y=y, z=z)
                        self.pose_msgs.append(self.target_relative_to_base(cmd_pose))
                        print(f'{BLU}Sample: {i},{j}{RST}')
                        print(f'\t{GRN}Z: {z}{RST}')
                        print(f'\t{GRN}Theta: {np.degrees(theta)}{RST}')
                        print(f'\t{GRN}Phi: {np.degrees(phi)}{RST}')
