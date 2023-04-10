#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from lu_vp_detect import VPDetection


class VPDetectorNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.vpd = VPDetection(length_thresh=30, principal_point=None, focal_length=None, seed=1337)
        self.camera_params_initialized = False

        camera_img_topic = rospy.get_param("~camera_img_topic", "/kitti/camera_color_left/image_raw")
        camera_info_topic = rospy.get_param("~camera_info_topic", "/kitti/camera_color_left/camera_info")
        visual_img_topic = rospy.get_param("~visual_img_topic", "/vp_detector/visual_img")
        self.output_visual_flag = rospy.get_param("~output_visual_flag", True)
        
        # Subscribe to image topic
        # self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.image_sub = rospy.Subscriber(camera_img_topic, Image, self.image_callback)
        if self.output_visual_flag:
            # Publish visual image topic
            self.visual_img_pub = rospy.Publisher(visual_img_topic, Image, queue_size=1)

        # Wait for camera info message and set camera parameters
        rospy.loginfo("Waiting for camera info message...")
        # msg = rospy.wait_for_message("/camera/camera_info", CameraInfo)
        msg = rospy.wait_for_message(camera_info_topic, CameraInfo)
        rospy.loginfo("Received camera info message")
        fx = msg.K[0]
        fy = msg.K[4]
        cx = msg.K[2]
        cy = msg.K[5]
        # self.vpd.principal_point = (cx, cy)
        self.vpd.focal_length = (fx + fy) / 2
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
        vps = self.vpd.find_vps(cv_image)

        # print(vps)

        # # Print and publish VP coordinates
        # for vp in vps:
        #     rospy.loginfo("VP coordinates: ({:.2f}, {:.2f}, {:.2f})".format(vp[0], vp[1], vp[2]))
        #     # Publish the VP coordinates on a topic
        #     # ...
        
        # if self.output_visual_flag:
        #     visual_img = self.vpd.create_debug_VP_image(show_image=False)
        #     visual_msg = self.bridge.cv2_to_imgmsg(visual_img, encoding="bgr8")
        #     # Publish the visual image on a topic
        #     self.visual_img_pub.publish(visual_msg)
            

        # for vp in vps_2D:
        #     # check this point is in the image
        #     if vp[0] < 0 or vp[0] > img_shape[1] or vp[1] < 0 or vp[1] > img_shape[0]:
        #         continue
        #     cv2.circle(img_cV2, (int(vp[0]), int(vp[1])), 5, (0, 0, 255), -1)
        #     cv2.putText(img_cV2, str(vp), (int(vp[0]), int(vp[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        #     print(vp)


        vps_2D = self.vpd.vps_2D

        if self.output_visual_flag:
            # visual_img = self.vpd.create_debug_VP_image(show_image=False)

            # draw the 2D vanishing points
            for vp in vps_2D:
                # check this point is in the image
                if vp[0] < 0 or vp[0] > cv_image.shape[1] or vp[1] < 0 or vp[1] > cv_image.shape[0]:
                    continue
                cv2.circle(cv_image, (int(vp[0]), int(vp[1])), 5, (0, 0, 255), -1)
                cv2.putText(cv_image, str(vp), (int(vp[0]), int(vp[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            visual_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            # Publish the visual image on a topic
            self.visual_img_pub.publish(visual_msg)


if __name__ == '__main__':
    rospy.init_node('vp_detector_node')
    vp_detector_node = VPDetectorNode()
    rospy.spin()
