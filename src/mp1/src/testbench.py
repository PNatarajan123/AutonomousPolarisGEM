import numpy as np
import cv2
from skimage import morphology


def gradient_thresh(img, thresh_min=25, thresh_max=100):
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
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    sobelx = cv2.Sobel(blur, cv2.CV_64F, 1, 0, ksize=3)
    sobely = cv2.Sobel(blur, cv2.CV_64F, 0, 1, ksize=3)
    abs_sobelx = np.absolute(sobelx)
    abs_sobely = np.absolute(sobely)
    scaled_sobelx = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
    scaled_sobely = np.uint8(255*abs_sobely/np.max(abs_sobely))
    grad_combined = cv2.addWeighted(scaled_sobelx, 0.5, scaled_sobely, 0.5, 0)

    binary_output = np.zeros_like(grad_combined)
    binary_output[(grad_combined >= thresh_min) & (grad_combined <= thresh_max)] = 1
    return binary_output

def color_thresh(img, s_thresh=(100, 255)):
    """
    Convert RGB to HSL and threshold to binary image to highlight white and yellow colors.
    
    Parameters:
    - img: Input image in RGB
    - s_thresh: Saturation channel threshold for selecting white.
    - white_thresh: Lightness channel threshold for selecting white.
    - yellow_h_thresh: Hue channel threshold for selecting yellow.
    
    Returns:
    - binary_output: Binary image highlighting the white and yellow lanes.
    """
    high_yellow = np.array([35,255,255])
    low_yellow = np.array([25,25,102])
    hsl_image = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
    whitefilter = cv2.inRange(hsl_image[:,:,1],180,255)
    yellowfilter = cv2.inRange(hsl_image, low_yellow, high_yellow)
    binary_output = cv2.bitwise_or(whitefilter,yellowfilter) / 255


    return binary_output

def combinedBinaryImage(img):
    """
    Get combined binary image from color filter and sobel filter
    """
    #1. Apply sobel filter and color filter on input image
    #2. Combine the outputs
    ## Here you can use as many methods as you want.

    ## TODO

    ####

    SobelOutput = gradient_thresh(img)
    ColorOutput = color_thresh(img)

    binaryImage = np.zeros_like(SobelOutput)
    binaryImage[(ColorOutput==1)|(SobelOutput==1)] = 1
    # Remove noise from binary image
    binaryImage = morphology.remove_small_objects(binaryImage.astype(np.bool_),min_size=50,connectivity=2)
    binaryImage = binaryImage.astype(np.uint8)

    return binaryImage

def perspective_transform(img, verbose=False):
    """
    Get bird's eye view from input image
    """
    #1. Visually determine 4 source points and 4 destination points
    #2. Get M, the transform matrix, and Minv, the inverse using cv2.getPerspectiveTransform()
    #3. Generate warped image in bird view using cv2.warpPerspective()

    ## TODO

    ####
    height, width = img.shape[:2]
    print(height)
    print(width)
    point1 = np.float32([[0.4*width,.6*height],[0,height],[0.6*width,.6*height],[width,height]])
    point2 = np.float32([[100, 0],[100, 640],[640, 0],[640, 640]])

    M = cv2.getPerspectiveTransform(point1,point2)
    Minv = np.linalg.inv(M)
    warped_img = cv2.warpPerspective(img,M,(800,600))
    print(warped_img.shape)

    return warped_img, M, Minv

'''
img = cv2.imread('test.png')
binary_output = gradient_thresh(img)
cv2.imshow('Original Image', img)
cv2.imshow('Gradient Threshold', binary_output * 255)
cv2.waitKey(0)
cv2.destroyAllWindows()
'''

img = cv2.imread('test.png')
# binary_output = combinedBinaryImage(img)
# binary_output = gradient_thresh(img)
binary_output,c,v = perspective_transform(img)
cv2.imshow('Original Image', img)
cv2.imshow('color_thresh', binary_output * 255)
cv2.waitKey(0)
cv2.destroyAllWindows()