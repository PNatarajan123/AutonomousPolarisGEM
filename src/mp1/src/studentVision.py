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
        #self.sub_image = rospy.Subscriber('/gem/front_single_camera/front_single_camera/image_raw', Image, self.img_callback, queue_size=1)
        # Uncomment this line for lane detection of videos in rosbag
        self.sub_image = rospy.Subscriber('camera/image_raw', Image, self.img_callback, queue_size=1)
        #self.sub_image = rospy.Subscriber('zed2/zed_node/rgb/image_rect_color', Image, self.img_callback, queue_size=1)

        self.pub_image = rospy.Publisher("lane_detection/annotate", Image, queue_size=1)
        self.pub_bird = rospy.Publisher("lane_detection/birdseye", Image, queue_size=1)
        self.left_line = Line(n=5)
        self.right_line = Line(n=5)
        self.detected = False
        self.hist = True


    def img_callback(self, data):
        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        raw_img = cv_image.copy()
        mask_image, bird_image = self.detection(raw_img)

        if mask_image is not None and bird_image is not None:
            # Convert an OpenCV image into a ROS image message
            out_img_msg = self.bridge.cv2_to_imgmsg(mask_image, 'bgr8')
            out_bird_msg = self.bridge.cv2_to_imgmsg(bird_image, 'bgr8')

            # Publish image message in ROS
            self.pub_image.publish(out_img_msg)
            self.pub_bird.publish(out_bird_msg)


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

        low_yellow = np.array([25,25,102])
        high_yellow = np.array([35,255,255])
        hsl_image = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        h, s, l = cv2.split(hsl_image)
        whitefilter = cv2.inRange(s, 150, 255)
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
        # print(warped_img.shape)

        return warped_img, M, Minv

    def detection(self, img):

        binary_img = self.combinedBinaryImage(img) * 255
        img_birdeye, M, Minv = self.perspective_transform(binary_img/255)

        binary_img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2RGB)
        out_bird_msg = self.bridge.cv2_to_imgmsg(binary_img, 'rgb8')

        # Publish image message in ROS
        # self.pub_bird.publish(out_bird_msg)

        if not self.hist:
            # Fit lane without previous result
            ret = line_fit(img_birdeye)
            left_fit = ret['left_fit']
            right_fit = ret['right_fit']
            nonzerox = ret['nonzerox']
            nonzeroy = ret['nonzeroy']
            left_lane_inds = ret['left_lane_inds']
            right_lane_inds = ret['right_lane_inds']

        else:
            # Fit lane with previous result
            if not self.detected:
                ret = line_fit(img_birdeye)

                if ret is not None:
                    left_fit = ret['left_fit']
                    right_fit = ret['right_fit']
                    nonzerox = ret['nonzerox']
                    nonzeroy = ret['nonzeroy']
                    left_lane_inds = ret['left_lane_inds']
                    right_lane_inds = ret['right_lane_inds']

                    left_fit = self.left_line.add_fit(left_fit)
                    right_fit = self.right_line.add_fit(right_fit)

                    self.detected = True

            else:
                left_fit = self.left_line.get_fit()
                right_fit = self.right_line.get_fit()
                ret = tune_fit(img_birdeye, left_fit, right_fit)

                if ret is not None:
                    left_fit = ret['left_fit']
                    right_fit = ret['right_fit']
                    nonzerox = ret['nonzerox']
                    nonzeroy = ret['nonzeroy']
                    left_lane_inds = ret['left_lane_inds']
                    right_lane_inds = ret['right_lane_inds']

                    left_fit = self.left_line.add_fit(left_fit)
                    right_fit = self.right_line.add_fit(right_fit)

                else:
                    self.detected = False

            # Annotate original image
            bird_fit_img = None
            combine_fit_img = None
            if ret is not None:
                bird_fit_img = bird_fit(img_birdeye, ret, save_file=None)
                combine_fit_img = final_viz(img, left_fit, right_fit, Minv)
            else:
                print("Unable to detect lanes")

            return combine_fit_img, bird_fit_img


if __name__ == '__main__':
    # init args
    rospy.init_node('lanenet_node', anonymous=True)
    lanenet_detector()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
