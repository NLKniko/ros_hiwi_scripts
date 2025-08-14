from interbotix_xs_modules.locobot import InterbotixLocobotXS

def main():
    bot = InterbotixLocobotXS(robot_model="locobot_wx250s", arm_model="mobile_wx250s", use_move_base_action=True)
    bot.camera.pan_tilt_move(0,0.2)
    #bot.arm.set_ee_pose_components(x=0.35, y=0, z=0.57, moving_time=1.5)
    bot.arm.set_joint_positions([0, 0.032, -1.282, 3.131, -0.618, -3.131])

if __name__=='__main__':
    main()