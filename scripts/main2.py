#!/usr/bin/python3

from data_generator2 import DataGenerator
import os
import rospy

if __name__ == "__main__":
    ros_node = rospy.init_node("data_generator", anonymous=True)
    dg = DataGenerator(os.path.join(os.path.dirname(__file__),'calibration_results.json'),
                                   os.path.join(os.path.dirname(__file__),'tact_calibration.json')
                                   )
    # dg.move_tactile_tip_to_target_with_z_offset(0.07)
    # dg.generate_data_collection_poses()
    dg.generate_poses()
    # dg.collect_data()
    while not rospy.is_shutdown():
        # print(dg.k.receive_transform(parent='charuco', frame='tactile')[0])
        rospy.sleep(0.25)
        pass

