#!/usr/bin/env python3

import mediapipe as mp
import numpy as np

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

class CameraPoseNode(DTROS):
    def __init__(self, node_name):
        super(CameraPoseNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # Get vehicle name and set up topics
        camera_topic = f"/mrduck/camera_node/image/compressed"
        #Throttled camera test
        # camera_topic = "/mrduck/camera_node/image/compressed_throttled"

        twist_topic = f"/mrduck/car_cmd_switch_node/cmd"
        
        # Bridge between ROS and OpenCV
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        
        # Movement parameters
        self.vel = 0.2
        # self.vel_angular = 0
        # self.stop = 0.0

        self.centroid = None
        self.pose = "None"

        self.cv_image = None
        self.processed_image = None

        # ROS subscribers and publishers
        self.subscriber = rospy.Subscriber(camera_topic, CompressedImage, self.camera_callback)
        self.publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

    def camera_callback(self, data):
        # shape is 480x640x3
        # self.cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
        try:
            # Convert the compressed image ROS message to OpenCV format
            self.cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")

            # Resize image
            scale_percent = 50  # percent of original size
            width = int(self.cv_image.shape[1] * scale_percent / 100)
            height = int(self.cv_image.shape[0] * scale_percent / 100)
            dim = (width, height)
            resized_image = cv2.resize(self.cv_image, dim, interpolation=cv2.INTER_AREA)

            # self.processed_image = resized_image

            self.cv_image = resized_image

            # if self.processed_image is not None and self.centroid is not None:
            if self.processed_image is not None:

                if self.centroid is not None:
                    # Draw vertical green line at centroid
                    height = self.processed_image.shape[0]
                    cv2.line(self.processed_image, (self.centroid, 0), (self.centroid, height), (0, 255, 0), 2)

                if self.pose is not None:

                    text_start = (10, 30)  # Position at (10, 30) pixels from the top-left corner
                    font_scale = 1
                    font_color = (0, 0, 255)  
                    font_thickness = 2
                    font = cv2.FONT_HERSHEY_SIMPLEX

                    # Put text onto the image
                    cv2.putText(self.processed_image, self.pose, text_start, font, font_scale, font_color, font_thickness)

        except:
            print("COULDN'T SCALE IMAGE")

            

        cv2.namedWindow('plotted_feed', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('plotted_feed', self.processed_image)

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
            
        return image, landmarks
    
    #Preprocess data
    def get_landmark_pos(self, landmarks, body_point):
        #HARDCODE FOR NOW
        # if self.processed_image is not None:
        #     img_width = self.processed_image.shape[1]
        #     img_height = self.processed_image.shape[0]
        # else:
        if landmarks is not None:
            img_width = 320
            img_height = 240

            # Convert string name to the corresponding PoseLandmark enum
            try:
                body_point_enum = getattr(mp_pose.PoseLandmark, body_point)
                landmark = landmarks[body_point_enum.value]

                x_pos = landmark.x * img_width
                y_pos = landmark.y * img_height

                # print(f"Landmark position for {body_point}: ({x_pos}, {y_pos})")
                return (x_pos, y_pos)
            except AttributeError:
                print(f"Error: Invalid landmark name '{body_point}'.")
                return (0, 0)  
            
        else:
            return (0,0)


    def pos_landmarks(self, landmarks):

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

        # #Test these individually, and check for proper corresponding movement.
        # print(f'right_wrist coord: {right_knee}')
        # print(f'left_wrist coord: {left_knee}')

        right_shoulder_angle = self.calc_angle(right_elbow, right_shoulder, right_hip)
        right_elbow_angle = self.calc_angle(right_wrist, right_elbow, right_shoulder)

        left_shoulder_angle = self.calc_angle(left_elbow, left_shoulder, left_hip)
        left_elbow_angle = self.calc_angle(left_wrist, left_elbow, left_shoulder)

        angle_tol = 20
        orientation_tol = 25
        angle_90 = 80
        angle_180 = 180

        #T pose for stop
        # if (self.angle_within_tol(right_shoulder_angle, angle_90, angle_tol)) and (self.angle_within_tol(left_shoulder_angle, angle_90, angle_tol))
        #     self.pose = "STOP"

        #PUT CONDITIONS IN LIST, THIS MAKES THE MOST SENSE I THINK
        stop_conditions = [
            self.check_angle(right_shoulder_angle, angle_90, angle_tol),
            self.check_angle(left_shoulder_angle, angle_90, angle_tol),
            self.check_orientation(right_shoulder, right_elbow, 'x', orientation_tol),
            self.check_orientation(left_shoulder, left_elbow, 'x', orientation_tol),
        ]

        forward_conditions = [
            self.check_angle(right_shoulder_angle, angle_180, angle_tol),
            self.check_angle(left_shoulder_angle, angle_180, angle_tol),
            self.check_orientation(right_shoulder, right_elbow, 'y', orientation_tol),
            self.check_orientation(left_shoulder, left_elbow, 'y', orientation_tol),
        ]

        #Hands on hips
        backward_conditions = [
            # self.check_angle(right_shoulder_angle, 50, angle_tol),
            # self.check_angle(left_shoulder_angle, 50, angle_tol),
            self.check_orientation(right_shoulder, right_wrist, 'y', orientation_tol),
            self.check_orientation(left_shoulder, left_wrist, 'y', orientation_tol),  
            self.check_angle(right_elbow_angle, 100, angle_tol),
            self.check_angle(left_elbow_angle, 100, angle_tol),

        ]

        #FROM HUMAN PERSPECTIVE
        right_90 = [
            self.check_angle(right_shoulder_angle, angle_90, angle_tol),
            # self.check_angle(left_shoulder_angle, 50, angle_tol),
            self.check_orientation(right_shoulder, right_elbow, 'x', orientation_tol),
            self.check_orientation(left_shoulder, left_wrist, 'y', orientation_tol),  
            self.check_angle(left_elbow_angle, 100, angle_tol),                    
        ]

        left_90 = [
            self.check_angle(left_shoulder_angle, angle_90, angle_tol),
            # self.check_angle(right_shoulder_angle, 50, angle_tol),
            self.check_orientation(left_shoulder, left_elbow, 'x', orientation_tol),
            self.check_orientation(right_shoulder, right_wrist, 'y', orientation_tol),  
            self.check_angle(right_elbow_angle, 100, angle_tol),                    
        ]

        # right_orientation = self.check_orientation(right_shoulder, right_wrist, 'y', orientation_tol),
        # left_orientation = self.check_orientation(left_shoulder, left_wrist, 'y', orientation_tol),

        # right_dist = self.check_dist(right_hip, right_wrist)
        # left_dist = self.check_dist(left_hip, left_wrist)

        #Check items in list
        # print(f'Shoulder angles - Left: {left_shoulder_angle} Right: {right_shoulder_angle}') # 60ish
        # print(f'Shoulder - Wrist Orientation: Left: {left_orientation} Right: {right_orientation}')

        # print(f'Elbow angles - Left: {left_elbow_angle} Right: {right_elbow_angle}') #90ish
        # print(f'Shoulder - Hip Distance: Left: {left_dist}, Right: {right_dist}') #~50

        # print(f'left_90: {left_90}')
        # print(f'right_90: {right_90}')
   

        if all(stop_conditions):
            self.pose = 'STOP'

        elif all(forward_conditions):
            self.pose = 'FORWARD'

        elif all(backward_conditions):
            self.pose = 'BACKWARD'
            
        elif all(left_90):
            self.pose = 'LEFT_90'

        elif all(right_90):
            self.pose = 'RIGHT_90'

        else:
            self.pose = "None"

    def calc_angle(self, first_joint, second_joint, third_joint):

        a = np.array(first_joint) # First
        b = np.array(second_joint) # Mid
        c = np.array(third_joint) # End
        
        radians = np.arctan2(c[1]-b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
        angle = np.abs(radians*180.0/np.pi)
        
        if angle > 180.0:
            angle = 360-angle
            
        return angle 
    
    def check_angle(self, angle, target, tolerance):
        return target - tolerance < angle < target + tolerance



    def check_orientation(self, landmark1, landmark2, axis, tolerance) :
        if axis == 'x':
            diff = abs(landmark1[1] - landmark2[1])
            return -tolerance < diff < tolerance
            #Convert to test function:
            # return diff
        if axis == 'y':
            diff =  abs(landmark1[0] - landmark2[0])
            return -tolerance < diff < tolerance
            #Convert to test function:
            # return diff


    def follow_turn(self, landmarks):
        #Need to add distance calc
        if landmarks is not None:
            #Store shoulder landmarks, use to calculate centroid. left/ right assumes that person is facing camera
            #Should be 640
            img_width = self.processed_image.shape[1]

            left_hip_pos = self.get_landmark_pos(landmarks, 'LEFT_HIP')[0]
            right_hip_pos = self.get_landmark_pos(landmarks, 'RIGHT_HIP')[0]


            self.centroid = int((left_hip_pos + right_hip_pos) / 2)

            #Should be positive on left side of screen, negative on right side of screen
            #Turn should be positive to turn right, negative to turn left

            # img_center = img_width / 2
            img_center = 160
            pos_diff = img_center - self.centroid
            #Find this empirically
            norm = 2.0

            turn = (pos_diff / img_center) * norm

            twist_msg = Twist2DStamped(v=self.vel, omega=turn)
            self.publisher.publish(twist_msg)
        else:
            self.centroid = None
            turn = 0

            twist_msg = Twist2DStamped(v=0.0, omega=turn) #0.2?
            self.publisher.publish(twist_msg)
        
        return turn


    def search(self):
        #Calc how long to turn to spin 45 deg
        omega_rad = 1.0
        target_angle = 45.0
        angle_to_rad = target_angle * (np.pi / 180)

        spin_time = angle_to_rad / omega_rad

        start_time = rospy.Time.now()
        # rate = rospy.Rate(10)
        while rospy.Time.now() - start_time < rospy.Duration(spin_time):
            twist_msg = Twist2DStamped(v=0, omega=omega_rad)
            self.publisher.publish(twist_msg)
            self.rate.sleep()

        twist_msg = Twist2DStamped(v=0, omega=0)
        self.publisher.publish(twist_msg)
        rospy.sleep(2)


    def run(self, v_input, omega_input):
        twist_msg = Twist2DStamped(v=v_input, omega=omega_input)
        self.publisher.publish(twist_msg)

    def stop(self):
        twist_msg = Twist2DStamped(v=0.0, omega=0.0)
        self.publisher.publish(twist_msg)
    
    def command_turn(self, target_angle, v, omega_rad):
        angle_to_rad = target_angle * (np.pi / 180)
        spin_time = angle_to_rad / omega_rad
        forward_time = 2
        total_time = spin_time + forward_time
        start_time = rospy.Time.now()

        while rospy.Time.now() - start_time < rospy.Duration(total_time):
            elapsed_time = rospy.Time.now() - start_time
            if elapsed_time < rospy.Duration(spin_time):
                twist_msg = Twist2DStamped(v=self.vel, omega=omega_rad)
            else:
                twist_msg = Twist2DStamped(v=self.vel, omega=0)
            self.publisher.publish(twist_msg)
            self.rate.sleep()

        # Stop the robot
        self.publisher.publish(Twist2DStamped(v=0, omega=0))
        rospy.sleep(0.5)
  

    def main(self):

        # rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.processed_image, landmarks = self.detect_pose(self.cv_image)

            # if landmarks is not None:
                # print('=========DETECTED============')

                # self.guidance_landmarks(landmarks)
            self.pos_landmarks(landmarks)
            
                # # if self.pose is not None:
                # if self.pose == 'STOP':
                #     self.stop()
                #     self.rate.sleep()

                # elif self.pose == 'FORWARD':
                #     self.run(self.vel, 0)
                #     self.rate.sleep()

                # elif self.pose == 'BACKWARD':
                #     self.run(-self.vel, 0)
                #     self.rate.sleep()
                
                # elif self.pose == 'LEFT_90':
                #     #Right turn from Robot Perspective
                #     self.command_turn(target_angle=90.0, v=self.vel, omega=-1.0)
                #     self.rate.sleep()

                # elif self.pose == 'RIGHT_90':
                #     #Left turn from Robot Perspective
                #     self.command_turn(target_angle=-90.0, v=self.vel, omega=1.0)
                #     self.rate.sleep()

                # else:

            self.follow_turn(landmarks)
            self.centroid = None
            # self.rate.sleep()
                # # rate.sleep()
                

            # else:
            if landmarks is None:
                self.search()
                self.pose = "None"
                self.centroid = None
            # else:
            #     self.pose = 'None'
            #     self.search()
            #     rospy.sleep(0.5)
            #     # continue
            #     # self.stop()


            self.rate.sleep()

 
if __name__ == '__main__':
    node = CameraPoseNode(node_name='camera_pose')
    node.main()
    rospy.spin()




