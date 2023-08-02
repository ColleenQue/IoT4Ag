import rospy
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.msg  
import numpy as np
from PIL import Image
i = 0
root = "/home/grasp/Downloads/ViBES-sensor_segment_colleen/ViBES-sensor_segment/depth_storage"

def convert_depth_image(ros_image):
    bridge = CvBridge()

    depth_image = bridge.imgmsg_to_cv2(ros_image)#desired_encoding="passthrough")

    depth_array = np.array(depth_image, dtype=np.float32)
    print(np.shape(depth_array))
    m1 = np.nanmin(depth_array)

    m2 = np.nanmax(depth_array)


    print(np.nanmin(depth_array))
    print(np.where(depth_array == m1))
    
    print(np.nanmax(depth_array))
    print(np.where(depth_array == m2))

    global i


    np.save('depth.npy', ros_image)
    im = Image.fromarray(depth_array)
    im = im.convert("L")
    idx = str(i).zfill(4)
    im.save(root+"/depth/frame{index}.png".format(index = idx))
    i += 1
    print("depth_idx: ", i)

def pixel2depth():
    rospy.init_node('pixel2depth',anonymous=True)
    rospy.Subscriber("/realsense/depth/image_rect_raw", sensor_msgs.msg.Image,callback=convert_depth_image, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    pixel2depth()