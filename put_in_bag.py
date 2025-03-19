from math import pi
import time
from interbotix_xs_modules.locobot import InterbotixLocobotXS
import rospy
from geometry_msgs.msg import Twist
from math import pi

# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx250s use_nav:=true use_lidar:=true rtabmap_args:=-d enable_pipeline:=true use_perception:=true'
# Then change to this directory and type 'python combo_control.py'


def main():
    bot = InterbotixLocobotXS(robot_model="locobot_wx250s", arm_model="mobile_wx250s", use_move_base_action=True)
    bot.camera.pan_tilt_move(0,0.2618)
    bot.base.move_to_pose(0.15, -2.5, 0.314, True)
    bot.camera.pan_tilt_move(0,0.75)
    success, clusters = bot.pcl.get_cluster_positions(ref_frame="locobot/arm_base_link", sort_axis="y", reverse=True)
    bot.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=1.5)
    bot.gripper.open()

    if len(clusters) > 0:
        for cluster in clusters:
            x, y, z = cluster["position"]

            # Move above the object
            bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
            bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.5)

            # Close gripper to grasp
            bot.gripper.close()
            time.sleep(1)

            # Move up after grasping
            bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.1, pitch=0.5)

            # Move to drop-off location (adjust as needed)
            drop_x, drop_y, drop_z = 0.4, -0.2, 0.1  # Example drop-off coordinates
            bot.arm.set_ee_pose_components(x=drop_x, y=drop_y, z=drop_z+0.05, pitch=-0.5)
            bot.arm.set_ee_pose_components(x=drop_x, y=drop_y, z=drop_z, pitch=-0.5)

            # Release the object
            bot.gripper.open()
            time.sleep(1)

            # Move back up before next cycle
            bot.arm.set_ee_pose_components(x=drop_x, y=drop_y, z=drop_z+0.1, pitch=-0.5)
        
        rospy.loginfo("All objects have been processed.")
    
    else:
        rospy.loginfo("No objects detected, returning home.")

    bot.camera.pan_tilt_move(0,0.2618)
    

    bot.base.move_to_pose(0, 0, -1.7, True)

    bot.arm.set_ee_pose_components(x=0.3, z=0.1, moving_time=1.5)
    bot.gripper.open()
    bot.camera.pan_tilt_move(0,0)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()