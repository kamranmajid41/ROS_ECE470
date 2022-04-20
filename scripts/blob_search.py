#!/usr/bin/env python
import sys
import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = 0.144
beta = 750.0
tx = 0.25
ty = 0.025

# Function that converts image coord to world coord
def IMG2W(col, row):

    x_c = (row - 240) / beta 
    y_c = (col - 320) / beta 

    R_z = np.array([[0.999, -0.003],[0.003,  0.999]])
    R_z_inv = np.linalg.inv(R_z)
    T = np.array([tx, ty])
    T = np.transpose(T)
    c = np.array([x_c, y_c])
    c = np.transpose(c)
    
    w = R_z_inv.dot(c + T)

    x_w = w[0]  
    y_w = w[1]

    return x_w, y_w, 0.1   

# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================
    
    # Filter by Color
    params.filterByColor = True

    # Filter by Area.
    params.filterByArea = False

    # Filter by Circularity
    params.filterByCircularity = False

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False


    # ========================= Student's code ends here ===========================
    # By color 
    params.blobColor = 255

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    if(color == "orange"):
        lower = (10,180,50)     
        upper = (40,255,255) 
    else:      # blue
        lower = (55,130,110)    
        upper = (190 ,255,255)

    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))
    
    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, np.array([]), (0, 0, 255))
    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw

