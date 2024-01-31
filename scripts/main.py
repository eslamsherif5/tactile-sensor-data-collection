#!/usr/bin/python3

# from data_generator import DataGenerator
# import os
# import rospy

# if __name__ == "__main__":
#     ros_node = rospy.init_node("data_generator", anonymous=True)
#     dg = DataGenerator(os.path.join(os.path.dirname(__file__),'calibration_results.json'),
#                                    os.path.join(os.path.dirname(__file__),'tact_calibration.json')
#                                    )
#     # dg.move_tactile_tip_to_target_with_z_offset(0.07)
#     # dg.generate_data_collection_poses()
#     dg.collect_data()
#     while not rospy.is_shutdown():
#         # print(dg.k.receive_transform(parent='charuco', frame='tactile')[0])
#         rospy.sleep(0.25)
#         pass

#!/usr/bin/python3
#

from cv2 import sqrt
import numpy as np
import sys
import math
import time
import rospy
from geometry_msgs.msg import Pose, Vector3, PoseStamped, Twist
from std_msgs.msg import Float64, Bool
from scipy.spatial.transform import Rotation as R
import copy
import random


class DataGenerator:
    
    def __init__(self):

        self.stop = False

        self.ros_node = rospy.init_node('data_generator', anonymous=True)
        self.contact_stats_pub = rospy.Publisher('contact_status', Bool, queue_size=10)
        self.contact_deg_pub = rospy.Publisher("contact_angle", Vector3, queue_size=2)
        self.sensor_depth_pub = rospy.Publisher("sensor_depth", Float64, queue_size=10)
        self.cmd_pose_pub = rospy.Publisher("/ur_cmd_pose", Pose, queue_size=2)
        self.cmd_vel_pub = rospy.Publisher("/ur_cmd_vel", Twist, queue_size=2)
        self.pose_subs = rospy.Subscriber("/dvs/pose", PoseStamped, self.pose_callback, queue_size=2)
        self.rate = rospy.Rate(100)

        #Define in plane frame
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

        #The angle ranges for rotation magnitude
        #self.angle_values =  [0, 0.06981317007977318, 0.13962634015954636, 0.20943951023931956] #[0.0174532925, 0.034906585, 0.0523598776]# [0.0174532925, 0.034906585, 0.0523598776, 0.075, 0.095, 0.115, 0.135, 0.15] #ENVTACT [0.0174532925, 0.034906585, 0.0523598776]# ENVTACT_depth# #[0.01, 0.0174, 0.0261] [0.075, 0.15]#
        self.angle_values =  [np.radians(10), np.radians(20)]#0.0174532925, 0.034906585, 0.0523598776]# [0.0174532925, 0.034906585, 0.0523598776, 0.075, 0.095, 0.115, 0.135, 0.15] #ENVTACT [0.0174532925, 0.034906585, 0.0523598776]# ENVTACT_depth# #[0.01, 0.0174, 0.0261] [0.075, 0.15]#
        
        # print(self.angle_values)
        #specify number of rotation directions
        self.N_examples = 4
        self.N_iterations_per_example = 3 #10

        #Specify z value for contact
        self.contact_thresh = 0.1#-0.0045
        self.min_z =  0.15#-0.0075 # -0.006 #for depth
        self.max_z = 0.11 #The Z value in the plane frame   

        B_AR = np.array([[0.0164194150157777, -0.999847614722551, -0.005928755701237023],
                         [0.9998521024990179, 0.016388570277986, 0.005214201044868802],
                         [-0.005116242647925262, -0.006013492984016437, 0.999968830495880]])

        self.C_T_rot = np.array([[0.7047522051166324, -0.7091562595361953, -0.020536040121596634],
                                 [0.7068215319126389, 0.7043344909655556, -0.06569814960074429],
                                 [0.061054495394806035, 0.03178560046504011, 0.9976281993784866]])
        
        self.C_T_rot = R.from_matrix(self.C_T_rot)
        
        self.C_T_pos = np.array([
            [-0.0006479281853649793],
            [0.000206590180011193],
            [0.07784053619032716]
        ])
        


        m1 = R.from_euler('z', 90, degrees=True).as_matrix()
        m2 = R.from_euler('x', 180, degrees=True).as_matrix()
        m = np.matmul(m1, m2)

        self.base_to_plane_rot =  R.from_matrix(np.matmul(B_AR, m)) 

        self.base_to_plane_pos = np.array([[0.709],
                                           [-0.186],
                                           [-0.04]])

        self.run_node()

    def pose_callback(self, pose_msg):
        self.robot_pose = pose_msg

        #Convert pose message from relative to base to relative to plane
        pose_relative_to_plane = np.matmul(np.transpose(self.base_to_plane_rot.as_matrix()), 
                                           -self.base_to_plane_pos + np.array([self.robot_pose.pose.position.x, self.robot_pose.pose.position.y, self.robot_pose.pose.position.z]).reshape((3,1)) )
        
        
        #TODO: Better method for determining contact (e.g. include 3D kinematics, use of laser tracker)
        if (-pose_relative_to_plane[2]) < self.contact_thresh:
            self.contact_status.data = True
            self.sensor_depth.data = pose_relative_to_plane[2] + self.contact_thresh
        else:
            self.contact_status.data = False
            self.sensor_depth.data = 0
            
        base_to_cam_rot = R.from_quat([pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w])

        #TODO: More precise method for determining contact (e.g. use of laser tracker)
        cam_from_default_rot = R.from_matrix( np.matmul(np.transpose(self.base_to_plane_rot.as_matrix()), base_to_cam_rot.as_matrix()))
        camera_from_default_rotvec = cam_from_default_rot.as_rotvec()
        self.contact_angle.x = camera_from_default_rotvec[0]
        self.contact_angle.y = camera_from_default_rotvec[1]
        self.contact_angle.z = camera_from_default_rotvec[2]

        self.contact_deg_pub.publish(self.contact_angle)
        self.contact_stats_pub.publish(self.contact_status)
        self.sensor_depth_pub.publish(self.sensor_depth)

    def plane_to_base(self, pose_msg):
        #plane position vector 
        pos_vector = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]).reshape((3,1))

        #convert to base frame
        base_pos_vector = self.base_to_plane_pos + np.matmul(self.base_to_plane_rot.as_matrix(), pos_vector)

        cmd_pose_msg = copy.deepcopy(pose_msg)
        cmd_pose_msg.position.x = base_pos_vector[0]
        cmd_pose_msg.position.y = base_pos_vector[1]
        cmd_pose_msg.position.z = base_pos_vector[2]

        return cmd_pose_msg

        
    def run_node(self):

        target_quat = self.base_to_plane_rot.as_quat()
        self.cmd_pose.orientation.x = target_quat[0]
        self.cmd_pose.orientation.y = target_quat[1]
        self.cmd_pose.orientation.z = target_quat[2]
        self.cmd_pose.orientation.w = target_quat[3]
        self.cmd_pose.position.z = -self.max_z
        print(self.plane_to_base(self.cmd_pose))
        input("wait")
        self.cmd_pose_pub.publish(self.plane_to_base(self.cmd_pose))
        rospy.sleep(5)
        
        for i in range(self.N_examples):
            if i==0:
                #The normal case
                rx = 0
                ry = 0
                rz = 0
                
                target_rot = R.from_matrix(np.matmul(self.base_to_plane_rot.as_matrix(), R.from_rotvec([rx, ry, rz]).as_matrix()))
                target_quat = target_rot.as_quat()
                self.cmd_pose.orientation.x = target_quat[0]
                self.cmd_pose.orientation.y = target_quat[1]
                self.cmd_pose.orientation.z = target_quat[2]
                self.cmd_pose.orientation.w = target_quat[3]

                for j in range(self.N_iterations_per_example):                    
                    self.cmd_pose.position.z = -self.max_z
                    self.cmd_pose_pub.publish(self.plane_to_base(self.cmd_pose))
                    rospy.sleep(2)

                    self.cmd_pose.position.z = -random.uniform(self.min_z, self.contact_thresh)# - 0.0072) # Used before finding the actual transformtaion from the camera to the tactile sensor
                    self.cmd_pose_pub.publish(self.plane_to_base(self.cmd_pose))
                    rospy.sleep(3)

            else:   
                #Any other angle
                theta = i * 2 * math.pi/(self.N_examples - 1)
                for phi in self.angle_values:
                    rx = phi * math.cos(theta)
                    ry = phi * math.sin(theta)
                    rz = 0
                    
                    target_rot = R.from_matrix(np.matmul(self.base_to_plane_rot.as_matrix(), R.from_rotvec([rx, ry, rz]).as_matrix()))
                    target_quat = target_rot.as_quat()
                    self.cmd_pose.orientation.x = target_quat[0]
                    self.cmd_pose.orientation.y = target_quat[1]
                    self.cmd_pose.orientation.z = target_quat[2]
                    self.cmd_pose.orientation.w = target_quat[3]

                    for j in range(self.N_iterations_per_example):
                        self.cmd_pose.position.z = -self.max_z
                        self.cmd_pose.position.x = self.org_pose.position.x
                        self.cmd_pose.position.y = self.org_pose.position.y
                        self.cmd_pose_pub.publish(self.plane_to_base(self.cmd_pose))
                        rospy.sleep(1)

                        #TODO: edit the z movement to move in camera frame Z direction not the plane frame
                        self.cmd_pose.position.z = -(random.uniform(self.min_z, self.contact_thresh)) #- 0.0072)   - (self.contact_thresh -  self.contact_thresh * math.cos(phi))) # Used before finding the actual transformtaion from the camera to the tactile sensor
                        self.cmd_pose.position.x = self.org_pose.position.x + math.tan(phi) * math.sin(theta) * (self.cmd_pose.position.z - self.max_z)
                        self.cmd_pose.position.y = self.org_pose.position.y + math.tan(phi) * math.cos(theta) * (self.cmd_pose.position.z - self.max_z)

                        # print(self.cmd_pose.position.z)
                        # print(self.plane_to_base(self.cmd_pose))
                        self.cmd_pose_pub.publish(self.plane_to_base(self.cmd_pose))
                        # self.cmd_vel.linear.z = 0.01
                        # self.cmd_vel_pub.publish(self.cmd_vel)
                        # while self.robot_pose.pose.position.z > self.contact_thresh:
                        #     rospy.sleep(0.05)
                        # self.cmd_vel.linear.z = 0.0
                        # self.cmd_vel_pub.publish(self.cmd_vel)
                        rospy.sleep(3)
                        
                        self.cmd_pose.position.z = -self.max_z
                        self.cmd_pose.position.x = self.org_pose.position.x
                        self.cmd_pose.position.y = self.org_pose.position.y
                        self.cmd_pose_pub.publish(self.plane_to_base(self.cmd_pose))
                        rospy.sleep(1)
        
if __name__ == '__main__':
    robot = DataGenerator()
    exit()
