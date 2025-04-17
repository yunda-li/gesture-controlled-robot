#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String, Int32
from duckietown_msgs.msg import Twist2DStamped
from duckietown.dtros import DTROS, NodeType



class MotorCommandNode(DTROS):
    def __init__(self, node_name):
        super(MotorCommandNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        self.pose_sub = rospy.Subscriber("/pose_info", String, self.pose_callback)
        #Int32 can't be None, must be 0
        self.centroid_sub = rospy.Subscriber('/centroid', Int32, self.centroid_callback)

        twist_topic = "/mrduck/car_cmd_switch_node/cmd"
        self.cmd_pub = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

        self.frame_without_landmarks_count = 0
        self.frames_threshold = 15

        self.pose = None
        self.centroid = 0
        self.rate = rospy.Rate(10)

        self.vel = 0.2

    def pose_callback(self, msg):
        self.pose = msg.data

    def centroid_callback(self, msg):
        self.centroid = msg.data
        rospy.loginfo(f'MOTOR COMMAND CENTROID: {self.centroid}')

    def stop(self):
        rospy.sleep(0.25)
        twist_msg = Twist2DStamped(v=0.0, omega=0.0)
        self.cmd_pub.publish(twist_msg)

    def run(self, v_input, omega_input):
        rospy.sleep(0.25)
        twist_msg = Twist2DStamped(v=v_input, omega=omega_input)
        self.cmd_pub.publish(twist_msg)
        rospy.sleep(0.25)
    

    def follow_turn(self, centroid):
        img_width = 640 * 0.75
        img_center = img_width / 2
        pos_diff = img_center - centroid
        norm = 2.0  # This should be adjusted based on actual turning needs and scaling

        # rospy.loginfo(f'Centroid: {self.centroid}, Image center: {img_center}, Position difference: {pos_diff}')

        turn = (pos_diff / img_center) * norm
        # rospy.loginfo(f'Calculated turn: {turn}, Velocity: {self.vel}')

        twist_msg = Twist2DStamped(v=self.vel, omega=turn)
        self.cmd_pub.publish(twist_msg)


    def command_turn(self, target_angle, v, omega_rad):
        angle_to_rad = target_angle * (np.pi / 180)
        omega_rad = abs(omega_rad) if target_angle > 0 else -abs(omega_rad)
        spin_time = angle_to_rad / omega_rad
        forward_time = 2
        total_time = spin_time + forward_time
        start_time = rospy.Time.now()

        while rospy.Time.now() - start_time < rospy.Duration(total_time):
            elapsed_time = rospy.Time.now() - start_time
            if elapsed_time < rospy.Duration(spin_time):
                twist_msg = Twist2DStamped(v=v, omega=omega_rad)
            else:
                twist_msg = Twist2DStamped(v=v, omega=0)
            self.cmd_pub.publish(twist_msg)
            self.rate.sleep()

        # Stop the robot
        twist_msg = Twist2DStamped(v=0, omega=0)
        self.cmd_pub.publish(twist_msg)
        rospy.sleep(0.5)

    def search(self):
        rospy.sleep(0.5)
        #Calc how long to turn to spin 45 deg
        omega_rad = 1.2
        target_angle = 45.0
        angle_to_rad = target_angle * (np.pi / 180)

        spin_time = angle_to_rad / omega_rad

        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(spin_time):
            twist_msg = Twist2DStamped(v=0, omega=omega_rad)
            self.cmd_pub.publish(twist_msg)
            self.rate.sleep()

        twist_msg = Twist2DStamped(v=0, omega=0)
        self.cmd_pub.publish(twist_msg)
        rospy.sleep(0.5)

    def main(self):
        search_delay_counter = 0
        search_delay_threshold = 5  # mess around with this?

        while not rospy.has_param('pose_processor_ready'):
            rospy.loginfo("Waiting for pose_processor to be ready...")
            rospy.sleep(0.2) 

        while not rospy.is_shutdown():
            if self.pose != 'None':
                if self.pose == 'STOP':
                    self.stop()
                    self.rate.sleep()
                    search_delay_counter = 0  
                    continue
                elif self.pose == 'FORWARD':
                    self.run(self.vel, 0)
                    self.rate.sleep()
                    search_delay_counter = 0 
                    continue
                elif self.pose == 'BACKWARD':
                    self.run(-self.vel, 0)
                    self.rate.sleep()
                    search_delay_counter = 0 
                    continue
                elif self.pose == 'LEFT_90':
                    self.command_turn(target_angle=-150.0, v=self.vel, omega_rad=-1.0)
                    self.rate.sleep()
                    search_delay_counter = 0  
                    continue
                elif self.pose == 'RIGHT_90':
                    self.command_turn(target_angle=90.0, v=self.vel, omega_rad=1.0)
                    self.rate.sleep()
                    search_delay_counter = 0  
                    continue

            if self.pose == 'None' and self.centroid > 0:
                self.follow_turn(self.centroid)
                self.centroid = 0
                search_delay_counter = 0  
            else:
                if self.frame_without_landmarks_count >= self.frames_threshold and search_delay_counter >= search_delay_threshold:
                    rospy.loginfo("Starting to search...")
                    self.search()
                    self.frame_without_landmarks_count = 0  
                    self.centroid = 0
                    search_delay_counter = 0  
                else:
                    self.frame_without_landmarks_count += 1
                    search_delay_counter += 1  

            self.rate.sleep()

if __name__ == '__main__':
    node = MotorCommandNode(node_name='motor_command')


    node.main()
    rospy.spin()

