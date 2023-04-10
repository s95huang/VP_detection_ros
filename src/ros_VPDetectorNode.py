#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from VPDetection import VPDetection # use the VPDetection.py in the same folder


class VPDetectorNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.vpd = VPDetection(reject_degree_th=15.0)
        self.camera_params_initialized = False

        camera_img_topic = rospy.get_param("~camera_img_topic", "/kitti/camera_color_left/image_raw")
        camera_info_topic = rospy.get_param("~camera_info_topic", "/kitti/camera_color_left/camera_info")
        visual_img_topic = rospy.get_param("~visual_img_topic", "/vp_detector/visual_img")
        self.output_visual_flag = rospy.get_param("~output_visual_flag", True)
        
        # Subscribe to image topic
        self.image_sub = rospy.Subscriber(camera_img_topic, Image, self.image_callback)
        if self.output_visual_flag:
            # Publish visual image topic
            self.visual_img_pub = rospy.Publisher(visual_img_topic, Image, queue_size=1)

        # Wait for camera info message and set camera parameters
        rospy.loginfo("Waiting for camera info message...")
        msg = rospy.wait_for_message(camera_info_topic, CameraInfo)
        rospy.loginfo("Received camera info message")
        fx = msg.K[0]
        fy = msg.K[4]
        cx = msg.K[2]
        cy = msg.K[5]
        self.camera_params_initialized = True
        rospy.loginfo("Camera parameters initialized: fx={}, fy={}, cx={}, cy={}".format(fx, fy, cx, cy))

    def image_callback(self, msg):
        if not self.camera_params_initialized:
            rospy.logwarn("Camera parameters not yet initialized. Skipping image.")
            return

        try:
            # Convert ROS Image message to OpenCV image
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv_image = img.copy()
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Detect vanishing points
        vanishing_point = self.vpd.find_vanishing_points(cv_image)

        if self.output_visual_flag and vanishing_point:
            # Draw the vanishing point
            cv2.circle(cv_image, (int(vanishing_point[0]), int(vanishing_point[1])), 5, (0, 0, 255), -1)
            cv2.putText(cv_image, str(vanishing_point), (int(vanishing_point[0]), int(vanishing_point[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            visual_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            # Publish the visual image on a topic
            self.visual_img_pub.publish(visual_msg)


if __name__ == '__main__':
    rospy.init_node('vp_detector_node')
    vp_detector_node = VPDetectorNode()
    rospy.spin()
