#!/usr/bin/env python
import tf
import copy
import rospy
import threading
from cartesian_state_msgs.msg import PoseTwist
from geometry_msgs.msg import Twist, Quaternion

class FakeCartesianStateSim:
    def __init__(self):
        frame_id = rospy.get_param("~frame_id", "")
        init_posx = rospy.get_param("~init_posx", 0.0)
        init_posy = rospy.get_param("~init_posy", 0.0)
        init_posz = rospy.get_param("~init_posz", 0.0)
        init_quatx = rospy.get_param("~init_quatx", 0.0)
        init_quaty = rospy.get_param("~init_quaty", 0.0)
        init_quatz = rospy.get_param("~init_quatz", 0.0)
        init_quatw = rospy.get_param("~init_quatw", 1.0)
        child_frame_id = rospy.get_param("~child_frame_id", "")
        state_pub_rate_hz = rospy.get_param("~state_pub_rate_hz", 125.0)
        input_topic = rospy.get_param("~input_topic", "/ur3_cartesian_velocity_controller/command_cart_vel")
        output_topic = rospy.get_param("~output_topic", "/ur3_cartesian_velocity_controller/ee_state")

        self.current_twist = Twist()
        self.lock = threading.Lock()

        self.state = PoseTwist()
        self.state.header.frame_id = frame_id
        self.state.child_frame_id = child_frame_id
        self.state.pose.position.x = init_posx
        self.state.pose.position.y = init_posy
        self.state.pose.position.z = init_posz
        self.state.pose.orientation.x = init_quatx
        self.state.pose.orientation.y = init_quaty
        self.state.pose.orientation.z = init_quatz
        self.state.pose.orientation.w = init_quatw

        self.last_time = rospy.Time.now()
        self.current_rpy = list(tf.transformations.euler_from_quaternion((init_quatx, init_quaty, init_quatz, init_quatw)))

        self.sub = rospy.Subscriber(input_topic, Twist, self.vel_callback)
        self.pub = rospy.Publisher(output_topic, PoseTwist, queue_size=1)

        self.state_timer = rospy.Timer(rospy.Duration(1.0/state_pub_rate_hz), self.state_publisher)

    def vel_callback(self, msg):
        self.lock.acquire()
        self.current_twist = copy.deepcopy(msg)
        self.lock.release()

    def state_publisher(self, timer_event):
        time_diff = (timer_event.current_real-self.last_time).to_sec()
        self.lock.acquire()
        self.state.pose.position.x += self.current_twist.linear.x * time_diff 
        self.state.pose.position.y += self.current_twist.linear.y * time_diff 
        self.state.pose.position.z += self.current_twist.linear.z * time_diff 
        self.current_rpy[0] += self.current_twist.angular.x * time_diff
        self.current_rpy[1] += self.current_twist.angular.y * time_diff
        self.current_rpy[2] += self.current_twist.angular.z * time_diff
        self.state.pose.orientation = Quaternion(*(tf.transformations.quaternion_from_euler(self.current_rpy[0], self.current_rpy[1], self.current_rpy[2])))
        self.state.twist = Twist(self.current_twist.linear, self.current_twist.angular)
        self.state.header.stamp = timer_event.current_real
        self.pub.publish(self.state)
        self.lock.release()
        self.last_time = timer_event.current_real

if __name__ == "__main__":
    rospy.init_node("fake_cartesian_state_sim")
    FakeCartesianStateSim()
    rospy.spin()
