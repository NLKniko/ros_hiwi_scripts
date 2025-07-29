from interbotix_xs_modules.locobot import InterbotixLocobotXS

def main():
    bot = InterbotixLocobotXS(robot_model="locobot_wx250s", arm_model="mobile_wx250s", use_move_base_action=True)
    bot.camera.pan_tilt_move(0,0.75)

if __name__=='__main__':
    main()