#!/usr/bin/env python3

import cv2
import mediapipe as mp
import numpy as np

import os
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
# rate = rospy.Rate(10)


class CameraPoseNode(DTROS):
    def __init__(self, node_name):
        super(CameraPoseNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # Get vehicle name and set up topics
        camera_topic = f"/mrduck/camera_node/image/compressed"
        twist_topic = f"/mrduck/car_cmd_switch_node/cmd"
        
        self.frame_without_landmarks_count = 0
        self.frames_threshold = 15

        # Bridge between ROS and OpenCV
        self.bridge = CvBridge()
        
        # Movement parameters
        self.vel_forward = 0.2


        self.centroid = None

        self.cv_image = None
        self.downsized_image = None
        self.processed_image = None

        # ROS Subscribers and Publishers
        self.subscriber = rospy.Subscriber(camera_topic, CompressedImage, self.camera_callback)
        self.publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

    def camera_callback(self, data):
        # shape is 480x640x3
        
        try:
            # Convert the compressed image ROS message to OpenCV format
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")

            # Resize the image to reduce resolution and computational load
            scale_percent = 75  # percent of original size
            width = int(cv_image.shape[1] * scale_percent / 100)
            height = int(cv_image.shape[0] * scale_percent / 100)
            dim = (width, height)
            resized_image = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)

            self.downsized_image = resized_image

            if self.processed_image is not None and self.centroid is not None:
                height = self.processed_image.shape[0]
                cv2.line(self.processed_image, (self.centroid, 0), (self.centroid, height), (0, 255, 0), 2)

            cv2.namedWindow('plotted_feed', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('plotted_feed', self.processed_image)

        except Exception as e:
            print(f"Error processing the image: {e}")

        cv2.waitKey(1)
    
    def detect_pose(self, image):

        # mp_drawing = mp.solutions.drawing_utils
        # mp_pose = mp.solutions.pose

        if image is None or image.size == 0:
            return False, None
    
        with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
            
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False

            # Make detection
            landmarks = None
            results = pose.process(image)

            # Recolor back to BGR
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            if results.pose_landmarks:
                mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                        mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2), 
                                        mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2))
                
                landmarks = results.pose_landmarks.landmark
                # print(f'LEFT ELBOW FOUND: {landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value]}')
                # print(f'LEFT WRIST FOUND: {landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value]}')

                #FORMAT:
                # LEFT ELBOW FOUND: x: 0.8109920620918274
                # y: 0.43222060799598694
                # z: 0.38565441966056824
                # visibility: 0.025279544293880463

            
        return image, landmarks


    def calc_turn(self, landmarks):
        if landmarks is not None:
            #Store shoulder landmarks, use to calculate centroid. left/ right assumes that person is facing camera
            #Should be 640
            img_width = self.processed_image.shape[1]

            left_hip_pos = landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].x * img_width
            right_hip_pos = landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].x * img_width

            # print(f'left shoulder: {left_hip_pos}')
            # print(f'right_shoulder: {right_hip_pos}')

            self.centroid = int((left_hip_pos + right_hip_pos) / 2)

            print(f'Centroid: {self.centroid}')


            #Should be positive on left side of screen, negative on right side of screen
            #Turn should be positive to turn right, negative to turn left

            img_center = img_width / 2
            pos_diff = img_center - self.centroid
            #Find this empirically
            norm = 1.8

            turn = (pos_diff / img_center) * norm

        else:
            self.centroid = None
            turn = 0

        return turn


    def search(self):
        #Calc how long to turn to spin 45 deg
        omega_rad = 1.0
        target_angle = 45.0
        angle_to_rad = target_angle * (np.pi / 180)

        spin_time = angle_to_rad / omega_rad

        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        while rospy.Time.now() - start_time < rospy.Duration(spin_time):
            twist_msg = Twist2DStamped(v=0, omega=omega_rad)
            self.publisher.publish(twist_msg)
            rate.sleep()

        twist_msg = Twist2DStamped(v=0, omega=0)
        self.publisher.publish(twist_msg)

        rospy.sleep(0.05)


    def run(self, v_input, omega_input):
        twist_msg = Twist2DStamped(v=v_input, omega=omega_input)
        self.publisher.publish(twist_msg)

    def stop(self):
        twist_msg = Twist2DStamped(v=0, omega=0)
        self.publisher.publish(twist_msg)

    def main(self):

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.processed_image, landmarks = self.detect_pose(self.downsized_image)

            if landmarks is not None:
                print('=========DETECTED============')

                turn = self.calc_turn(landmarks)
                #Should be positive on left side of screen, negative on right side of screen
                #Turn should be positive to turn right, negative to turn left

                print(f'calculated turn: {turn}')
                twist_msg = Twist2DStamped(v=self.vel_forward, omega=turn)
                self.publisher.publish(twist_msg)
                self.centroid = None

            
            else:
                if self.frame_without_landmarks_count >= self.frames_threshold:
                    self.search()
                    self.frame_without_landmarks_count = 0  # Reset the counter
                else:
                    self.frame_without_landmarks_count += 1


            rate.sleep()

 
if __name__ == '__main__':
    node = CameraPoseNode(node_name='camera_pose')
    node.main()
    rospy.spin()




