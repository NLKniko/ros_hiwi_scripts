import math
import time
from interbotix_xs_modules.locobot import InterbotixLocobotXS
import rospy
from geometry_msgs.msg import Twist

# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx250s use_nav:=true use_lidar:=true rtabmap_args:=-d enable_pipeline:=true use_perception:=true'
# Then change to this directory and type 'python3 put_in_bag.py'


def main():
    bot = InterbotixLocobotXS(robot_model="locobot_wx250s", arm_model="mobile_wx250s", use_move_base_action=True)
    bot.camera.pan_tilt_move(0,0.2618)
    
    # definition of the cargo's position
    cargo_pos = input("Please enter the x,y coordinates of the cargo: ")
    cargo_x_str, cargo_y_str = cargo_pos.split(',')
    cargo_pos_x = float(cargo_x_str)
    cargo_pos_y = float(cargo_y_str)
    rotation_cargo = float(input("Please enter the rotation of cargo's orientation: ")) # counterclockwise, 3.14 represents 180 degree.
    
    # definition of the drop-off position
    drop_pos = input("Please enter the x,y,z coordinates of the drop-off position: ")
    drop_x_str, drop_y_str, drop_z_str = drop_pos.split(',')
    drop_x = float(drop_x_str)
    drop_y = float(drop_y_str)
    drop_z = float(drop_z_str)
    rotation_drop = float(input("Please enter the rotation of the locobot in the direction of the drop-off orientation: ")) # counterclockwise, 3.14 represents 180 degree.

    # move the robot to the cargo's position
    bot.base.move_to_pose(cargo_pos_x, cargo_pos_y, rotation_cargo, True)

    # start the perception using realsense
    bot.camera.pan_tilt_move(0,0.75)
    success, clusters = bot.pcl.get_cluster_positions(ref_frame="locobot/arm_base_link", sort_axis="y", reverse=True)

    if len(clusters) > 0:
        for cluster in clusters:
            x, y, z = cluster["position"]

            bot.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=1.5)
            bot.gripper.open()

            # Move above the object
            bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
            bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.5)

            # Close gripper to grasp
            bot.gripper.close()
            time.sleep(1)
            bot.camera.pan_tilt_move(0,0)

            # Move up after grasping
            bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=-0.5)
            bot.arm.go_to_home_pose()
            bot.arm.go_to_sleep_pose()

            # Move to drop-off location (adjust as needed)
            bot.camera.pan_tilt_move(0,0.2618)
            bot.base.move_to_pose(drop_x, drop_y, rotation_drop+1.57, True) # set arm to rotate in order to avoid obstacle
            bot.camera.pan_tilt_move(0,0)
            bot.arm.set_ee_pose_components(x=0.35, y=0, z=drop_z, moving_time=1.5)
            bot.base.move_to_pose(drop_x, drop_y, rotation_drop, True)

            # Release the object
            bot.gripper.open()
            bot.base.move_to_pose(drop_x, drop_y, 3.14, True)
            bot.arm.go_to_home_pose()
            bot.arm.go_to_sleep_pose()

            # Move back up before next cycle
            bot.camera.pan_tilt_move(0,0.2618)
            bot.base.move_to_pose(cargo_pos_x, cargo_pos_y, rotation_cargo, True)
            bot.camera.pan_tilt_move(0,0.75)
            bot.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=1.5)
            bot.gripper.open()
        
        rospy.loginfo("All objects have been processed.")
        bot.arm.go_to_home_pose()
        bot.arm.go_to_sleep_pose()
    
    else:
        rospy.loginfo("No objects detected, returning home.")

    bot.camera.pan_tilt_move(0,0.2618)
    

    bot.base.move_to_pose(0, 0, 0, True)

    bot.camera.pan_tilt_move(0,0)

if __name__=='__main__':
    main()