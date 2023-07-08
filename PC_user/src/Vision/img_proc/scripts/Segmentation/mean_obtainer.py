#!/usr/bin/env python

import cv2
import numpy as np

def mouse_callback(event, x, y, flags, param):
    global img_original, img_copy, img_roi, x1, y1, x2, y2
    global Fil, roi_hsv, img_hsv, img_and_b, img_and_g
    global img_and_r, img_bck, img_comp
    if event == cv2.EVENT_LBUTTONDOWN:
        x1 = x
        y1 = y

    elif event == cv2.EVENT_LBUTTONUP:
        x2 = x
        y2 = y
        cv2.rectangle(img_copy, (x1,y1), (x2,y2), (0,255,0),2)
        img_roi = img_hsv[y1:y2, x1:x2]
        mean_x = cv2.mean(img_roi)
        img_b, img_g, img_r = cv2.split(img_hsv)
        
        up_h = mean_x[0] + delta_x
        lw_h = mean_x[0] - delta_x
        up_s = mean_x[1] + delta_x
        lw_s = mean_x[1] - delta_x
        up_v = mean_x[2] + delta_x
        lw_v = mean_x[2] - delta_x
        up = (up_h,up_s,up_v,0.0)
        lw = (lw_h,lw_s,lw_v,0.0)
        print(up_s)
        print(lw_s)
        print(delta_x)
        print("Mean \n")
        print(mean_x)
        Fil = cv2.inRange(img_hsv, lw, up)
    elif event == cv2.EVENT_RBUTTONDOWN:
        img_copy = img_original.copy()
        img_roi   = img_original.copy()
        Fil = img_original.copy()

def trackbar_callback(val):
    global delta_x
    delta_x = val
        
def main():
    global img_original, img_copy, img_roi, delta_x
    global Fil, roi_hsv, img_hsv, img_bck, img_comp

    img_original = cv2.imread('Base_Silver_I1.jpeg')
    img_copy   = img_original.copy()
    img_roi   = img_original.copy()
    img_hsv = cv2.cvtColor(img_copy, cv2.COLOR_BGR2HSV)
    roi_hsv    = cv2.cvtColor(img_roi, cv2.COLOR_BGR2HSV)
    Fil = img_roi.copy()
    img_comp = img_roi.copy()

    delta_x = 50
    cv2.namedWindow('Original')
    cv2.setMouseCallback('Original', mouse_callback)
    cv2.createTrackbar('Tolerancia','Original',delta_x, 100, trackbar_callback)

    while True:
        cv2.imshow("Original", img_copy)
        cv2.imshow("Filtered", Fil)
        if cv2.waitKey(100) & 0xFF == 27:
            break
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

