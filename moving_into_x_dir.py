import rospy
from geometry_msgs.msg import Twist
from math import pi
import time

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
            
if __name__ == '__main__':
    try:
        locobot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")