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
    bot.base.move_to_pose(0.2, -1, 0.5, True)
    bot.camera.pan_tilt_move(0,0.75)
    success, clusters = bot.pcl.get_cluster_positions(ref_frame="locobot/arm_base_link", sort_axis="y", reverse=True)
    bot.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=1.5)
    bot.gripper.open()

    #for cluster in clusters:
    for cluster in clusters:
        x, y, z = cluster["position"]
        bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
        bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.5)
        bot.gripper.close()

    bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
    bot.arm.go_to_home_pose()

    bot.base.move_to_pose(0, 0, 0, True)

    bot.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=1.5)
    bot.arm.set_ee_pose_components(x=x, y=y, z=z-0.05, pitch=0.5)
    bot.gripper.open()
    bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()