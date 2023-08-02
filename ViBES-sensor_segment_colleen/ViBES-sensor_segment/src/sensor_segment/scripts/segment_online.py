import cv2 as cv
import rospy
import numpy as np
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
from sensor_segment.msg import PixelLocation
from sensor_segment.msg import PixelLocationList
from sensor_segment.msg import ContourSize
from sensor_segment.msg import ContourSizeList

class SensorSegmenter:
    def __init__(self):
        print("[SensorSegmenter Node] test Initializing ...")
        rospy.init_node("sensor_segmtr", anonymous=False)
        self.rate = rospy.Rate(5)
        
        self.cv_bridge = CvBridge()
        self.img = None
        
        # tuning parameters 
        # pink/red: -0.5, yellow
        #python3 segment.py red.png -1.5 -1 0.33 0.48 20 5000 
        self.threshold_min = -1.5# -0.5
        self.threshold_max = -1
        self.sensor_area_min   = 20# 150
        self.sensor_area_max = 500000
        
        # subscribers
        self.img_sub = rospy.Subscriber(
                #"/camera/image_color",
                "/realsense/color/image_raw",
                Image,
                self.img_callback,
                queue_size=1,
                tcp_nodelay=True)    
        
        # publishers      
        self.yuv_thresholded_pub = rospy.Publisher(
                "/debug_img_yuv",
                Image,
                queue_size=1,
                )

        self.final_image_pub = rospy.Publisher(
                "/final_image",
                Image,
                queue_size=1,
                )
        
        self.pixel_loc_pub = rospy.Publisher(
                "/pixel_loc", 
                PixelLocationList,
                queue_size=1,
                )
        
        self.contour_pub = rospy.Publisher(
            "/contour_size",
            ContourSizeList,
            queue_size=1,
        )
        



        
    def img_callback(self, img_msg):    
        print("subscribing.........")
        self.img = self.cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
        self.parse_img()

    def parse_img(self):
        self.image_bgr, self.image_rgb = self.get_images()
        self.image_yuv, self.activation_u, self.activation_v = self.convert_to_yuv()
        self.image_rgb_filtered = self.apply_thresholds()
        self.yuv_thresholded_pub.publish(self.cv_bridge.cv2_to_imgmsg(self.image_rgb_filtered, "rgb8"))
        self.contours = self.get_contours()
        self.sensor_pixel_locs, self.image_rgb_filtered = self.get_sensor_pixel_locs()
        self.print_areas()
        self.final_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(self.image_rgb_filtered, "rgb8"))
            
    def get_images(self):
        image_bgr = self.img
        image_rgb = cv.cvtColor(image_bgr, cv.COLOR_BGR2RGB)

        return image_bgr, image_rgb 

    def convert_to_yuv(self):
        image_yuv = cv.cvtColor(self.image_bgr, cv.COLOR_BGR2YUV)

        # transforms image to yuv coordinates in wiki for intuitive thresholding, where limits are (-0.5, 0.5)
        axis_limits = -0.5
        image_yuv = (image_yuv/np.max(image_yuv)) + axis_limits
    
        # create kernel to segment by color
        bg_filt = axis_limits * np.ones((3, 3))
  
        # get convolved image for U and V channel
        activation_u = cv.filter2D(src=image_yuv[:, :, 1], ddepth=-1, kernel=bg_filt)
        activation_v = cv.filter2D(src=image_yuv[:, :, -1], ddepth=-1, kernel=bg_filt)

        return image_yuv, activation_u, activation_v
            
    def apply_thresholds(self):
        # based on colormap, should easily be able to identify a self.threshold_val for colored paper
        # we will likely use red colored paper in the demo, so we will just threshold with the v channel 
                
        # self.activation_v[self.activation_v > self.threshold_val ] = 0
        # self.activation_v[self.activation_v < self.threshold_val ] = 1
        

        self.activation_v[(self.activation_v < self.threshold_min) | (self.activation_v > self.threshold_max) ] = 0
        self.activation_v[(self.activation_v > self.threshold_min) & (self.activation_v < self.threshold_max)] = 1


        image_bgr_filtered = np.zeros_like(self.image_bgr)
    
        # apply filter
        for channel in range(image_bgr_filtered.shape[2]):
            image_bgr_filtered[:, :, channel] = self.image_bgr[:, :, channel]*self.activation_v
        
        image_bgr_filtered = image_bgr_filtered.astype(np.uint8)
        image_rgb_filtered = cv.cvtColor(image_bgr_filtered, cv.COLOR_BGR2RGB)
            
        return image_rgb_filtered
    
    def get_contours(self):
        image_gray = cv.cvtColor(self.image_rgb_filtered, cv.COLOR_RGB2GRAY)

        edge = cv.Canny(image_gray, 60, 180)

        contours, hierarchy = cv.findContours(image_gray, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
        contours = sorted(contours, key=lambda x: cv.contourArea(x), reverse=True)

        return contours 
    
    def print_areas(self):
        ''' For thresholding 'sensor_area' ''' 
        areas = []
        for c in self.contours:
            areas.append(cv.contourArea(c))
            #print(cv.contourArea(c)) # for debugging

        self.publish_contour(areas)
            
    def get_sensor_pixel_locs(self):
        
        # another thresholded value. For the demo, we can correlate the plant size 
        # or the width of colored blob in image to this thresholded value, knowing 
        # given the sensors will always be roughly the same size
        
        sensor_pixel_locs = [] 
        
        for i, c in enumerate(self.contours):
            if cv.contourArea(c) > self.sensor_area_min and cv.contourArea(c) < self.sensor_area_max:
                     
                # find centroid of convex hull
                M = cv.moments(c)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                sensor_pixel_locs.append([cx, cy])
                
                # on plot, print 'sensor' next to sensor with the colour detected 
                string_print = '(' + str(cx) + ', ' + str(cy) + ')'
                self.image_rgb_filtered = cv.putText(self.image_rgb_filtered, string_print, (cx + 50, cy + 50), fontFace = cv.FONT_HERSHEY_PLAIN, \
                color=(int(self.image_rgb[cy][cx][0]), int(self.image_rgb[cy][cx][1]), int(self.image_rgb[cy][cx][2])), thickness=2, fontScale=2) # self.image_rgb[cy][cx]/255
                
                # draw convex hull
                hull = cv.convexHull(c)
                self.image_rgb_filtered =  cv.drawContours(self.image_rgb_filtered, [hull], 0, (0, 255, 0), 2)
                
                pixel_loc = [cx, cy]
                
        self.publish_pixel_locs(sensor_pixel_locs)
         
        return sensor_pixel_locs, self.image_rgb_filtered

    def publish_pixel_locs(self, sensor_pixel_locs):
        pixel_loc_list_msg = PixelLocationList()
        
        for i in range(len(sensor_pixel_locs)):
            pixel_loc_xy_msg = PixelLocation()

            pixel_loc_xy_msg.pixel_location_xy = sensor_pixel_locs[i]
            pixel_loc_list_msg.pixel_location_list.append(pixel_loc_xy_msg)

        print('Pixel Locations:', pixel_loc_list_msg)
        self.pixel_loc_pub.publish(pixel_loc_list_msg)


    def publish_contour(self,areas):
        # contour_msg = ContourSize()
        # contour_msg.contour_size = areas[0]
        # print('Contour Area: ', contour_msg)
        # self.contour_pub.publish(contour_msg)


        contour_msg_list = ContourSizeList()
        
        for i in range(len(areas)):
            contour_msg = ContourSize()

            contour_msg.contour_size = areas[i]
            contour_msg_list.contour_size_list.append(contour_msg)

        print('Contour Areas:', contour_msg_list)
        self.contour_pub.publish(contour_msg_list)




if __name__ == "__main__":
    node = SensorSegmenter()
    while not rospy.is_shutdown():
        node.rate.sleep()

