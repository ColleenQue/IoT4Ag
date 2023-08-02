#!/usr/bin/env python2

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision_ctl.msg import ThrustHeading

class VisionControl:
    def __init__(self):
        print("[VisionControl Node] Initializing...")
        rospy.init_node("vision_ctl", anonymous=False)
        self.rate = rospy.Rate(5)

        self.cv_bridge = CvBridge()
        self.img = None

        # parameters
        self.des_yuv = np.array([rospy.get_param('~des_yuv_y'), rospy.get_param('~des_yuv_u'), rospy.get_param('~des_yuv_v')]) #(40, 160, 110)
        self.pm_range_yuv = np.array([rospy.get_param('~pm_range_yuv_y'), rospy.get_param('~pm_range_yuv_u'), rospy.get_param('~pm_range_yuv_v')]) #(30, 20, 20)
        self.desired_blob_center = np.array([rospy.get_param('~desired_blob_center_x'), rospy.get_param('~desired_blob_center_y')]) #(0.5, 0.5)
        self.desired_blob_width = rospy.get_param('~desired_blob_width') #0.40
        self.print_center_yuv = bool(rospy.get_param('~print_center_yuv')) #(40, 160, 110)
        
        # subscribers
        self.img_sub = rospy.Subscriber(
            "/camera/image_raw",
            Image,
            self.img_callback,
            queue_size=1,
            tcp_nodelay=True,
        )

        # publishers
        self.debugimg_pub = rospy.Publisher(
            "/debug_img",
            Image,
            queue_size=1,
        )
        self.ctl_pub = rospy.Publisher(
            "/ctl_cmd",
            ThrustHeading,
            queue_size=1,
        )

        self.cmd_msg = None

        self.object_not_found = True

        self.blob_center = None

    def img_callback(self, img_msg):
        self.img = self.cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
        # self.parse_img()

    def parse_img(self):
        # self.img_mask = np.logical_and(self.img[:, :, 1]>100, self.img[:, :, 2]<150)*255
        # self.img_mask = self.img[..., 1]

        if True:

            if self.img is None:
                return

            start_time = rospy.Time.now().to_sec()

            # convert to yuv
            self.img_yuv = cv2.cvtColor(self.img, cv2.COLOR_BGR2YUV)

            ##########################
            ### PRINT CENTER POINT ###
            ##########################
            if self.print_center_yuv:
                print('center point value')
                print(self.img_yuv[self.img_yuv.shape[0]//2, self.img_yuv.shape[1]//2, :])

            self.img_mask = cv2.inRange(self.img_yuv, tuple(self.des_yuv-self.pm_range_yuv), tuple(self.des_yuv+self.pm_range_yuv))
            
            self.img_mask = cv2.erode(self.img_mask, np.ones((8, 8), np.uint8))
            self.img_mask = cv2.dilate(self.img_mask, np.ones((20, 20), np.uint8))
            
            # extract median indexof nonzero values of img_mask
            nonzero = np.nonzero(self.img_mask)

            if nonzero[0].shape[0] > 0:

                self.object_not_found = False

                nonzero_median_i = int(np.median(nonzero[0]))
                nonzero_median_j = int(np.median(nonzero[1]))
                cv2.circle(self.img_mask, (nonzero_median_j, nonzero_median_i), 50, 128, thickness=20)

                # find center and width of blob
                self.blob_center = (nonzero_median_j/float(self.img_mask.shape[1]), nonzero_median_i/float(self.img_mask.shape[0]))
                self.blob_width = (nonzero[1].max() - nonzero[1].min()) / float(self.img_mask.shape[1])

                # print('blob center = {:.3f}, {:.3f}'.format(self.blob_center[0], self.blob_center[1]))
                # print('blob width = {:.3f}'.format(self.blob_width))

            else:

                self.object_not_found = True

                print('[VISION_TO_LOWCTL] Object not found!')

            self.img_mask = self.img_mask.astype(np.uint8)
            self.debugimg_pub.publish(self.cv_bridge.cv2_to_imgmsg(self.img_mask, "mono8"))

        # fake_thrust = (int(rospy.Time().now().to_sec()) % 10) / 10.
        self.publish_ctl()

        end_time = rospy.Time.now().to_sec()
        # print('{:.4f}'.format(end_time-start_time))

    def publish_ctl(self, fake_thrust=0.0):
        self.cmd_msg = ThrustHeading()
        if fake_thrust == 0.0 and self.blob_center is not None:
            self.cmd_msg.heading = 0.7 * (self.blob_center[0] - self.desired_blob_center[0]) / 0.5
            self.cmd_msg.thrust = 3 * (self.desired_blob_width - self.blob_width)#, 0, 1.0)
        else:
            print('[VISION_TO_LOWCTL] Fake thrust! blob center is {}'.format(self.blob_center))
            self.cmd_msg.heading = 0.0
            self.cmd_msg.thrust = fake_thrust
        self.ctl_pub.publish(self.cmd_msg)

if __name__ == "__main__":

    node = VisionControl()
    while not rospy.is_shutdown():
        node.parse_img()
        node.rate.sleep()
