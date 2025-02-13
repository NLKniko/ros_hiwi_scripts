import math
import time
from interbotix_xs_modules.locobot import InterbotixLocobotXS
import rospy
from geometry_msgs.msg import Twist
from math import pi

class locobot():
    def __init__(self):
        rospy.init_node("locobot_move")
        rospy.loginfo("Press CTL+C to terminate")
        self.publisher = rospy.Publisher("locobot/cmd_vel", Twist, queue_size=1)
        self.msg = Twist()
        rospy.on_shutdown(self.shutdown)
        self.rate = rospy.Rate(10)
        self.run()
        rospy.spin()
    
    def publish(self, msg_type="move"):
        while self.publisher.get_num_connections()<1:
            rospy.loginfo("Waiting for connection to publisher...")
            time.sleep(1)

        rospy.loginfo("connected")
        rospy.loginfo("Publishing %s message..." % msg_type)
        self.publisher.publish(self.msg)


    def run(self):
        self.msg.linear.x = 0.2
        self.publish()
        time.sleep(10)
        rospy.signal_shutdown("We are done here!")


    def shutdown(self):
        self.msg.linear.x = 0
        self.publish("stop")

def main():
    bot = InterbotixLocobotXS("locobot_wx250s", arm_model="mobile_wx250s")
    bot.camera.pan_tilt_move(0,0.75)
    success, clusters = bot.pcl.get_cluster_position(ref_frame="locobot/arm_base_link", sort_axis="y", reverse=True)
    bot.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=1.5)
    bot.gripper.open()