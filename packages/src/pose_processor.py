#!/usr/bin/env python3

import rospy
import cv2
import mediapipe as mp
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, Int32

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose


class PoseProcessorNode(DTROS):
    def __init__(self, node_name) -> None:
        super(PoseProcessorNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)

        self.pose = "None"
        self.centroid = None
        self.pose_pub = rospy.Publisher("/pose_info", String, queue_size=10)
        self.centroid_pub = rospy.Publisher("/centroid", Int32, queue_size=10)

        self.camera_sub = rospy.Subscriber("/mrduck/camera_node/image/compressed", CompressedImage, self.camera_callback)

        self.cv_image = None
        self.downsized_image = None
        self.processed_image = None

    def camera_callback(self, data) -> None:
        # Default shape is 480x640x3
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")

            # Need to balance performance increase with pose detection accuracy
            scale_percent = 75  # percent of original size
            width = int(cv_image.shape[1] * scale_percent / 100)
            height = int(cv_image.shape[0] * scale_percent / 100)
            dim = (width, height)
            resized_image = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)

            self.downsized_image = resized_image

            if self.processed_image is not None and self.centroid is not None:
                height = self.processed_image.shape[0]
                cv2.line(self.processed_image, (self.centroid, 0), (self.centroid, height), (0, 255, 0), 2)

            if self.processed_image is not None:

                if self.centroid is not None:
                    # Draw a vertical green line for human readability
                    height = self.processed_image.shape[0]
                    cv2.line(self.processed_image, (self.centroid, 0), (self.centroid, height), (0, 255, 0), 2)

                if self.pose is not None:
                    text_start = (10, 30)  
                    font_scale = 1
                    font_color = (0, 0, 255)  
                    font_thickness = 2
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(self.processed_image, self.pose, text_start, font, font_scale, font_color, font_thickness)

            cv2.namedWindow('plotted_feed', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('plotted_feed', self.processed_image)

        except Exception as e:
            print(f"Error processing the image: {e}")

        cv2.waitKey(1)
    


    def detect_pose(self, image):
        if image is None or image.size == 0:
            return False, None
    
        with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
            
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False

            landmarks = None
            results = pose.process(image)

            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            if results.pose_landmarks:
                mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                        mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2), 
                                        mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2))
                
                landmarks = results.pose_landmarks.landmark
            
        return image, landmarks
    
    def get_landmark_pos(self, landmarks, body_point) -> None:
        if landmarks is not None:
            img_width = 640 * .75
            img_height = 480 * .75

            try:
                body_point_enum = getattr(mp_pose.PoseLandmark, body_point)
                landmark = landmarks[body_point_enum.value]

                x_pos = landmark.x * img_width
                y_pos = landmark.y * img_height

                return (x_pos, y_pos)
            except AttributeError:
                print(f"Error: Invalid landmark name '{body_point}'.")
                return (0, 0)  
            
        else:
            pass


    def pos_landmarks(self, landmarks) -> None:

        landmark_positions = {name: self.get_landmark_pos(landmarks, name) for name in [
            'RIGHT_WRIST', 'LEFT_WRIST', 'RIGHT_ELBOW', 'LEFT_ELBOW', 
            'RIGHT_SHOULDER', 'LEFT_SHOULDER', 'RIGHT_HIP', 'LEFT_HIP'
        ]}

        right_wrist = landmark_positions['RIGHT_WRIST']
        left_wrist = landmark_positions['LEFT_WRIST']

        right_elbow = landmark_positions['RIGHT_ELBOW']
        left_elbow = landmark_positions['LEFT_ELBOW']

        right_shoulder = landmark_positions['RIGHT_SHOULDER']
        left_shoulder = landmark_positions['LEFT_SHOULDER']

        right_hip = landmark_positions['RIGHT_HIP']
        left_hip = landmark_positions['LEFT_HIP']


        right_shoulder_angle = self.calc_angle(right_elbow, right_shoulder, right_hip)
        right_elbow_angle = self.calc_angle(right_wrist, right_elbow, right_shoulder)

        left_shoulder_angle = self.calc_angle(left_elbow, left_shoulder, left_hip)
        left_elbow_angle = self.calc_angle(left_wrist, left_elbow, left_shoulder)

        angle_tol = 20
        orientation_tol = 25
        angle_90 = 80
        angle_180 = 180

        # T Pose
        stop_conditions = [
            self.check_angle(right_shoulder_angle, angle_90, angle_tol),
            self.check_angle(left_shoulder_angle, angle_90, angle_tol),
            self.check_orientation(right_shoulder, right_elbow, 'x', orientation_tol),
            self.check_orientation(left_shoulder, left_elbow, 'x', orientation_tol),
        ]

        # Arms up
        forward_conditions = [
            self.check_angle(right_shoulder_angle, angle_180, angle_tol),
            self.check_angle(left_shoulder_angle, angle_180, angle_tol),
            self.check_orientation(right_shoulder, right_elbow, 'y', orientation_tol),
            self.check_orientation(left_shoulder, left_elbow, 'y', orientation_tol),
        ]

        # Hands on hips
        backward_conditions = [
            self.check_orientation(right_shoulder, right_wrist, 'y', orientation_tol),
            self.check_orientation(left_shoulder, left_wrist, 'y', orientation_tol),  
            self.check_angle(right_elbow_angle, 100, angle_tol),
            self.check_angle(left_elbow_angle, 100, angle_tol),

        ]

        # Right from human perspective, Left for Robot
        right_90 = [
            self.check_angle(right_shoulder_angle, angle_90, angle_tol),
            self.check_orientation(right_shoulder, right_elbow, 'x', orientation_tol),
            self.check_orientation(left_shoulder, left_wrist, 'y', orientation_tol),  
            self.check_angle(left_elbow_angle, 100, angle_tol),                    
        ]

        # Left from human perspective, Right for Robot
        left_90 = [
            self.check_angle(left_shoulder_angle, angle_90, angle_tol),
            self.check_orientation(left_shoulder, left_elbow, 'x', orientation_tol),
            self.check_orientation(right_shoulder, right_wrist, 'y', orientation_tol),  
            self.check_angle(right_elbow_angle, 100, angle_tol),                    
        ]

        if all(stop_conditions):
            self.pose = 'STOP'
            self.pose_pub.publish(self.pose)

        elif all(forward_conditions):
            self.pose = 'FORWARD'
            self.pose_pub.publish(self.pose)

        elif all(backward_conditions):
            self.pose = 'BACKWARD'
            self.pose_pub.publish(self.pose)

        elif all(left_90):
            self.pose = 'LEFT_90'
            self.pose_pub.publish(self.pose)

        elif all(right_90):
            self.pose = 'RIGHT_90'
            self.pose_pub.publish(self.pose)

        else:
            self.pose = "None"
            self.pose_pub.publish(self.pose)

    def calc_angle(self, first_joint, second_joint, third_joint) -> float:

        a = np.array(first_joint) # First
        b = np.array(second_joint) # Middle
        c = np.array(third_joint) # End
        
        radians = np.arctan2(c[1]-b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
        angle = np.abs(radians*180.0/np.pi)
        
        if angle > 180.0:
            angle = 360-angle
            
        return angle 
    
    def check_angle(self, angle, target, tolerance) -> bool:
        return target - tolerance < angle < target + tolerance

    def check_orientation(self, landmark1, landmark2, axis, tolerance) -> bool:
        if axis == 'x':
            diff = abs(landmark1[1] - landmark2[1])
            return -tolerance < diff < tolerance

        if axis == 'y':
            diff =  abs(landmark1[0] - landmark2[0])
            return -tolerance < diff < tolerance


    def calc_centroid(self, landmarks) -> None:
        if landmarks is not None:
            #Store shoulder landmarks, use to calculate centroid. left/ right assumes that person is facing camera  

            left_hip_pos = self.get_landmark_pos(landmarks, 'LEFT_HIP')[0]
            right_hip_pos = self.get_landmark_pos(landmarks, 'RIGHT_HIP')[0]

            self.centroid = int((left_hip_pos + right_hip_pos) / 2)
            self.centroid_pub.publish(self.centroid)

    def main(self):
        rospy.set_param('pose_processor_ready', True)

        while not rospy.is_shutdown():

            self.processed_image, landmarks = self.detect_pose(self.downsized_image)

            self.calc_centroid(landmarks)

            if landmarks is not None:
                self.pos_landmarks(landmarks)


        self.rate.sleep()

if __name__ == '__main__':
    node = PoseProcessorNode(node_name='pose_processor')
    node.main()
    rospy.spin()
