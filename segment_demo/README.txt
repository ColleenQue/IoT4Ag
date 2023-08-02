=================================================================
=================================================================
How to run segment.py:
=================================================================
=================================================================
========================OPTION 1==================================
If you prefer not to use command line, comment argparser lines out, and uncomment the last 5 lines in segment.py
Should return the pixel location of the centroid of the detected sensors.
========================OPTION 2==================================
If running in command line, these are the threshold values that work well for the two images in the folder. 
Simply run the following:

-  For frame0040.jpg
$ python segment.py frame0040.jpg -0.5 3000
-  For frame0019.jpg
$ python segment.py frame0019.jpg -0.25 1000

=================================================================
=================================================================
More details:
=================================================================
=================================================================
- These threshold values are for red colored sensors.
- $ python segment.py -h:
	
 segment.py [-h] filename threshold_value sensor_area

********* Sensor Segmenter and Labeler *********

positional arguments:
  filename         String. Name of file we wish to segment sensors from.
  threshold_value  Float. Threshold value for V channel. For red, likely around: -0.5
  sensor_area      Integer. Threshold area for convex hull. If 0.5 m from sensor around: 3000; if 1 m from sensor around: 1000

options:
  -h, --help       show this help message and exit