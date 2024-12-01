import cv2
import numpy as np

def apply_laplacian(img):
    img = cv2.Laplacian(img, cv2.CV_64F)   
    img = img.astype(np.uint8)
    return img

def apply_thresholding(img):
    # img = img.astype(np.uint8)   # UNCOMMENT WHEN LAPLACIAN IS NOT USED
    img = cv2.resize(img, (600, 600), interpolation=cv2.INTER_AREA)   #600, 600
    h, w, _ = img.shape

    # Convert image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Create a mask for non-black pixels
    mask = cv2.threshold(gray, 50, 100, cv2.THRESH_BINARY)[1]  #best 30, 150
    # mask = cv2.threshold(gray, 20, 150, cv2.THRESH_BINARY)[1]

    # Replace non-black pixels with white
    img[mask > 0] = (255, 255, 255)

    return img, h, w, _

def apply_dilation(img, h, w):

    kernel = np.ones((2, 1), np.uint8)                #(4, 1) is best
    # kernel = np.ones((6, 1), np.uint8)                #(4, 1) is best

    img_dilation = cv2.dilate(img, kernel, iterations=1)

    img1 = img.copy()
    # img_dilation = cv2.line(img_dilation, (0, int(h * 0.645)), (w-1, int(h * 0.645)), (0, 255, 255), 2)
    # img_dilation = cv2.line(img_dilation, (0, int(h * 0.645) + 25), (w-1, int(h * 0.645) + 25), (0, 255, 255), 2)

    return img_dilation

    