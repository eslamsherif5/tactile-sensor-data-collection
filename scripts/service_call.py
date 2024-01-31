
#!/usr/bin/python3
import rospy
from tactile_data_collection.srv import moveRobot,desiredTCP,moveRobotRelative
import numpy as np
from scipy.spatial.transform import Rotation as sp_rot
from geometry_msgs.msg import Pose, Quaternion, Point

moveRobotCall = rospy.ServiceProxy('move_ur', moveRobot)  
adjustTCPCall = rospy.ServiceProxy('move_TCP', moveRobot)
relativeTCPCall = rospy.ServiceProxy('move_ur_relative', moveRobotRelative)

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
relativeTCPCall('tactile', '11', target_Pose_tactile)