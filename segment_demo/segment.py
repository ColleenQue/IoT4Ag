import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
import os
import glob
import sys
import argparse 

"""
# for poster:
# plot_without_axis = False 
# if plot_without_axis:
#     plt.box('off')
#     plt.axis('off')

"""

class SensorSegmenter():
    def __init__(self, args, argparser_used):
        if argparser_used:

            self.threshold_val_v_min = args['threshold_value_v_min']
            self.threshold_val_v_max = args['threshold_value_v_max']
            self.threshold_val_u_min = args['threshold_value_u_min']
            self.threshold_val_u_max = args['threshold_value_u_max']
            self.sensor_area1 = args['sensor_area1']
            self.sensor_area2 = args['sensor_area2']
            self.filename = args['filename']
        else:
            self.threshold_val, self.sensor_area, self.filename = args 
            

        #added 
        self.sensor_pixel_locs = []


        self.image_bgr, self.image_rgb = self.plot_rgb()
        self.image_yuv, self.activation_u, self.activation_v = self.convert_to_yuv()
        self.image_rgb_filtered_v = self.apply_thresholds(self.activation_v,threshold_min=self.threshold_val_v_min,threshold_max=self.threshold_val_v_max)
        self.image_rbg_filtered_u = self.apply_thresholds(self.activation_u,threshold_min=self.threshold_val_u_min,threshold_max=self.threshold_val_u_max)
        
        self.contours_v= self.get_contours(self.image_rgb_filtered_v )
        self.contours_u = self.get_contours(self.image_rbg_filtered_u)
        # self.sensor_pixel_locs = self.get_sensor_pixel_locs()
                
    def plot_rgb(self):
        image_bgr = cv.imread(self.filename)
        image_rgb = cv.cvtColor(image_bgr, cv.COLOR_BGR2RGB)
        plt.imshow(image_rgb)
        plt.title("Original RGB Image")

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
        plt.figure(figsize=(6,4))
        plt.title("U Channel")
        plt.imshow(activation_u, cmap='PiYG_r')
        plt.colorbar()
        plt.clim(-1.5, 1.5)
        activation_v = cv.filter2D(src=image_yuv[:, :, -1], ddepth=-1, kernel=bg_filt)
        plt.figure(figsize=(6,4))
        plt.title('V Channel')
        plt.imshow(activation_v, cmap='GnBu')
        plt.colorbar()
        plt.clim(-1.5, 1.5)
        
        return image_yuv, activation_u, activation_v
            
    def apply_thresholds(self, activation,threshold_min=-1.5,threshold_max = 1.5):
        # based on colormap, should easily be able to identify a self.threshold_val for colored paper
        # we will likely use red colored paper in the demo, so we will just threshold with the v channel 
                
        #activate = 1: only those with values less than threshold
       
        #activation[True] = 0?

        activation[(activation < threshold_min) | (activation > threshold_max) ] = 0
        activation[(activation > threshold_min) & (activation < threshold_max)] = 1


     
        image_bgr_filtered = np.zeros_like(self.image_yuv)
    
        # apply filter
        for channel in range(image_bgr_filtered.shape[2]):
            image_bgr_filtered[:, :, channel] = self.image_bgr[:, :, channel]*activation
        
        image_bgr_filtered = image_bgr_filtered.astype(np.uint8)
        image_rgb_filtered = cv.cvtColor(image_bgr_filtered, cv.COLOR_BGR2RGB)
    
        plt.figure()
        plt.imshow(image_rgb_filtered)
        plt.title('Segmented Sensors')
        filename = 'segmented.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        
        return image_rgb_filtered


    def get_contours(self,img):
        image_gray = cv.cvtColor(img, cv.COLOR_RGB2GRAY)

        edge = cv.Canny(image_gray, 60, 180)
        fig, ax = plt.subplots(1, figsize=(6,4))
        plt.box('tight')
        plt.title('Sensor Edges')
        plt.imshow(edge, cmap='Greys')

        contours, hierarchy = cv.findContours(image_gray, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
        contours = sorted(contours, key=lambda x: cv.contourArea(x), reverse=True)

        return contours 
        
        
    
    def print_areas(self,contour):
        ''' For thresholding 'sensor_area' ''' 
        areas = []
        for c in contour:
            areas.append(cv.contourArea(c))
            print(cv.contourArea(c)) # for debugging
            
    def get_sensor_pixel_locs(self,contour,img):
        fig, ax = plt.subplots(1, figsize=(6,4))
        
        # another thresholded value. For the demo, we can correlate the plant size 
        # or the width of colored blob in image to this thresholded value, knowing 
        # given the sensors will always be roughly the same size
        
        #sensor_pixel_locs = []
        for c in contour:
            if cv.contourArea(c) > self.sensor_area1 and cv.contourArea(c) < self.sensor_area2:
                                
                # find centroid of convex hull
                M = cv.moments(c)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                self.sensor_pixel_locs.append([cx, cy])
                
                # on plot, print 'sensor' next to sensor with the colour detected 
                ax.text(cx + 8, cy + 8, "sensor", color=list(self.image_rgb[cy][cx]/255), fontweight=20, fontsize=20)
                
                # draw convex hull
                hull = cv.convexHull(c)
                cv.drawContours(img, [hull], 0, (0, 255, 0), 2)
                #cv.drawContours(self.image_rgb_filtered, [hull], 0, (0, 255, 0), 2)
                    
        plt.imshow(img)
        plt.title('Segmented & Labeled by Sensor Color')
        plt.savefig("segmented_labeled.png", dpi = 300, bbox_inches='tight')
        
        return self.sensor_pixel_locs


def main(args,argparser_used):
    my_segmenter = SensorSegmenter(args, argparser_used)

    #V
    sensor_pixel_locs_v = my_segmenter.get_sensor_pixel_locs(my_segmenter.contours_v,my_segmenter.image_rgb_filtered_v)
    print("Total Sensors Detected:", len(sensor_pixel_locs_v))  
    print("Pixel Locations:", sensor_pixel_locs_v)
    plt.show()

    #U
    # my_segmenter2 = SensorSegmenter(args, argparser_used)
    # sensor_pixel_locs_u = my_segmenter2.get_sensor_pixel_locs(my_segmenter2.contours_u,my_segmenter2.image_rbg_filtered_u)
    # print("Total Sensors Detected:", len(sensor_pixel_locs_u))    
    # print("Pixel Locations:", sensor_pixel_locs_u)
    # plt.show()
    
if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description=('********* Sensor Segmenter and Labeler ********* \n'))
    
    # positional arguments
    parser.add_argument('filename', type=str,
                        help='String. Name of file we wish to segment sensors from.')
    parser.add_argument('threshold_value_v_min', type=float,
                        help='Float. Minimum Threshold value for V channel. ')
    
    parser.add_argument('threshold_value_v_max', type=float,
                        help='Float. Maximum Threshold value for V channel. ')
                        
    parser.add_argument('threshold_value_u_min', type=float,
                        help='Float. Threshold value for U channel. ')
    
    parser.add_argument('threshold_value_u_max', type=float,
                        help='Float. Threshold value for U channel. ')
    
    parser.add_argument('sensor_area1', type=int,
                        help='Integer. Threshold area (min) for convex hull. If 0.5 m from sensor around: 3000; if 1 m from sensor around: 1000')
                        
    parser.add_argument('sensor_area2', type=int,
                        help='Integer. Threshold area (max) for convex hull. If 0.5 m from sensor around: 3000; if 1 m from sensor around: 1000')
    args = vars(parser.parse_args())
    main(args,True)
    
    
    ''' If the command line isn't used, you can comment out the above, and uncomment this:'''
    # ## two different sets of examples to try
    # threshold_val = -0.5        # -0.5 | -0.25
    # sensor_area = 3000          # 3000 | 1000
    # filename = "frame0040.jpg"  # "frame0040.jpg" | "frame0019.jpg" 
    # args = (threshold_val, sensor_area1, sensor_area2, filename)
    # main(args, False)
