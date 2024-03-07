import time
import math
import numpy as np
import cv2
import rospy

from line_fit import line_fit, tune_fit, bird_fit, final_viz
from Line import Line
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from skimage import morphology



class lanenet_detector():
    def __init__(self):

        self.bridge = CvBridge()
        # NOTE
        # Uncomment this line for lane detection of GEM car in Gazebo
        self.sub_image = rospy.Subscriber('/gem/front_single_camera/front_single_camera/image_raw', Image, self.img_callback, queue_size=1)
        # Uncomment this line for lane detection of videos in rosbag
        self.pub_combined_binary = rospy.Publisher("lane_detection/combined_binary", Image, queue_size=1)
        self.pub_perspective = rospy.Publisher("lane_detection/perspective", Image, queue_size=1)


    def img_callback(self, data):
        raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")


        # Combined Binary
        combined_binary_img = self.combinedBinaryImage(raw_image) * 255
        combined_binary_img = cv2.cvtColor(combined_binary_img, cv2.COLOR_GRAY2BGR)


        perspective_img,m,minv = self.perspective_transform(raw_image)

        combined_binary_img_ros = self.bridge.cv2_to_imgmsg(combined_binary_img, "bgr8")
        self.pub_combined_binary.publish(combined_binary_img_ros)

        perspective_img_ros = self.bridge.cv2_to_imgmsg(perspective_img, "bgr8")
        self.pub_perspective.publish(perspective_img_ros)
    



    def gradient_thresh(self, img, thresh_min=25, thresh_max=100):
        """
        Apply sobel edge detection on input image in x, y direction
        """
        #1. Convert the image to gray scale
        #2. Gaussian blur the image
        #3. Use cv2.Sobel() to find derievatives for both X and Y Axis
        #4. Use cv2.addWeighted() to combine the results
        #5. Convert each pixel to unint8, then apply threshold to get binary image

        ## TODO

        ####
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), cv2.BORDER_DEFAULT)
        sobelx = cv2.Sobel(blur, cv2.CV_16S, 1, 0, ksize=3)
        sobely = cv2.Sobel(blur, cv2.CV_16S, 0, 1, ksize=3)
        abs_sobelx = np.absolute(sobelx)
        abs_sobely = np.absolute(sobely)
        scaled_sobelx = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
        scaled_sobely = np.uint8(255*abs_sobely/np.max(abs_sobely))
        grad_combined = cv2.addWeighted(scaled_sobelx, 0.5, scaled_sobely, 0.5, 0)
        binary_output = np.zeros_like(grad_combined)
        binary_output[(grad_combined >= thresh_min) & (grad_combined <= thresh_max)] = 1

        return binary_output


    def color_thresh(self, img, thresh=(100, 255)):
        """
        Convert RGB to HSL and threshold to binary image using S channel
        """
        #1. Convert the image from RGB to HSL
        #2. Apply threshold on S channel to get binary image
        #Hint: threshold on H to remove green grass
        ## TODO

        ####

        high_yellow = np.array([35,255,255])
        low_yellow = np.array([25,25,102])
        hsl_image = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        whitefilter = cv2.inRange(hsl_image[:,:,1],180,255)
        yellowfilter = cv2.inRange(hsl_image, low_yellow, high_yellow)
        binary_output = cv2.bitwise_or(whitefilter,yellowfilter) / 255

        return binary_output



    def combinedBinaryImage(self, img):
        """
        Get combined binary image from color filter and sobel filter
        """
        #1. Apply sobel filter and color filter on input image
        #2. Combine the outputs
        ## Here you can use as many methods as you want.

        ## TODO

        ####

        SobelOutput = self.gradient_thresh(img)
        ColorOutput = self.color_thresh(img)

        binaryImage = np.zeros_like(SobelOutput)
        binaryImage[(ColorOutput==1)|(SobelOutput==1)] = 1
        # Remove noise from binary image
        binaryImage = morphology.remove_small_objects(binaryImage.astype('bool'),min_size=50,connectivity=2)
        binaryImage = binaryImage.astype(np.uint8)
        # print(type(binaryImage))

        return binaryImage

    def perspective_transform(self, img, verbose=False):
        """
        Get bird's eye view from input image
        """
        #1. Visually determine 4 source points and 4 destination points
        #2. Get M, the transform matrix, and Minv, the inverse using cv2.getPerspectiveTransform()
        #3. Generate warped image in bird view using cv2.warpPerspective()

        ## TODO
        ####
        height, width = img.shape[:2]
        # print(height)
        # print(width)
        point1 = np.float32([[0.4*width,.6*height],[0,height],[0.6*width,.6*height],[width,height]])
        point2 = np.float32([[145, 0],[145, 605],[645, 0],[645, 605]])

        M = cv2.getPerspectiveTransform(point1,point2)
        Minv = np.linalg.inv(M)
        warped_img = cv2.warpPerspective(img,M,(800,600))
        warped_img[:,:100] = 0
        warped_img[:,700:] = 0
        # print(warped_img.shape)

        return warped_img, M, Minv


if __name__ == '__main__':
    # init args
    rospy.init_node('lanenet_node', anonymous=True)
    lanenet_detector()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
