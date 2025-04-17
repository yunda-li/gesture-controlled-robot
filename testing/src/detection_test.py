#!/usr/bin/env python3

import os
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped

class ColorDetectAndMoveNode(DTROS):
    def __init__(self, node_name):
        super(ColorDetectAndMoveNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        print(os.getcwd())
        # Get vehicle name and set up topics
        vehicle_name = 'mrduck'
        camera_topic = f"/{vehicle_name}/camera_node/image/compressed"
        twist_topic = f"/{vehicle_name}/car_cmd_switch_node/cmd"
        
        # Bridge between ROS and OpenCV
        self.bridge = CvBridge()
        
        # Movement parameters
        self.vel_forward = 0.1
        self.vel_angular = 0
        self.stop = 0.0

        self.match_pos = 320 #Change this to correspond to img width

        self.cv_image = None

        template_path = os.path.join('packages', 'duck_template.png')
        self.template = cv2.imread(template_path)
        assert self.template is not None, "Template could not be read"

        # ROS Subscribers and Publishers
        self.subscriber = rospy.Subscriber(camera_topic, CompressedImage, self.camera_callback)

        self.publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

    def camera_callback(self, data):
        # Maybe do some erosion and dilation if needed. I think its OK for now.
        # shape is 480x640x3
        self.cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
        # print(self.cv_image.shape)
        masked_image = self.convert_image(self.cv_image)

        cv2.namedWindow('camera_feed', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('camera_feed', self.cv_image)

        cv2.namedWindow('masked_feed', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('masked_feed', masked_image)

        # cv2.namedWindow('masked_feed', cv2.WINDOW_AUTOSIZE)
        cv2.waitKey(1)
 

    def detect_match(self, img):
        #Fix detection, add flags
        if img is None or img.size == 0:
            return False

        # img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = self.convert_image(img)
        template = self.convert_image(self.template)

        res = cv2.matchTemplate(img, template, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(res)
        # print(max_val)

        # print(max_loc[1])
        # self.match_pos = max_loc[0]


        if max_val > 0.6:  
            # print('DUCKIE DETECTED')
            # print(f'Duckie Location {max_loc}')
            # print(max_loc[0]): w, h
            self.match_pos = max_loc[0]

            return True
        else:
            return False
        
    def convert_image(self, img):
        lower_bound = np.array([0, 0, 100]) # B and G are less than your max, R is above your min
        upper_bound = np.array([100, 160, 255]) # B max, G max, R can go up to 255

        masked_img = cv2.inRange(img, lower_bound, upper_bound)

        return masked_img


    def stop(self):
        twist_msg = Twist2DStamped(v=0, omega=0)
        self.publisher.publish(twist_msg)

    def calc_turn(self):
        #Should be 640
        img_width = self.cv_image.shape[1]

        center = img_width / 2

        #Should be positive on left side of screen
        #negative on right side of screen
        pos_diff = center - self.match_pos
        print(f'match_pos: {self.match_pos}')
        print(f'pos_diff: {pos_diff}')
        #NEED TO FIND MAX TURN, NORMALIZE DISTANCE IN IMAGE TO ANGULAR VEL
        #Or at least, how much angular vel to reasonably reach edge of image?
        norm = 5.0

        #Turn should be positive to turn right, negative to turn left
        turn = (pos_diff / center) * norm
        
        print(f'Angular Vel input: {turn}')

        return turn

    def move(self):
        # Publish Twist message to move the robot
        #check the rostopic
        #Not consistently moving.
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.detect_match(self.cv_image):
                twist_msg = Twist2DStamped(v=self.vel_forward, omega=self.calc_turn())
                self.publisher.publish(twist_msg)
                # pass
                # print(f'Angular Vel input: {self.calc_turn()}')
            else:
                twist_msg = Twist2DStamped(v=0, omega=0)
                self.publisher.publish(twist_msg)

            rate.sleep()

if __name__ == '__main__':
    # rospy.init_node('color_detect_and_move_node', anonymous=False)
    node = ColorDetectAndMoveNode(node_name='detection_test')
    node.move()
    rospy.spin()
