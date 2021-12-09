#!/usr/bin/env python

import imutils
import cv2

def grab_contours(cnts):
    # if the length the contours tuple returned by cv2.findContours
    # is '2' then we are using either OpenCV v2.4, v4-beta, or
    # v4-official
    if len(cnts) == 2:
        cnts = cnts[0]

    # if the length of the contours tuple is '3' then we are using
    # either OpenCV v3, v4-pre, or v4-alpha
    elif len(cnts) == 3:
        cnts = cnts[1]

    # otherwise OpenCV has changed their cv2.findContours return
    # signature yet again and I have no idea WTH is going on
    else:
        raise Exception(("Contours tuple must have length 2 or 3, "
            "otherwise OpenCV changed their cv2.findContours return "
            "signature yet again. Refer to OpenCV's documentation "
            "in that case"))

    # return the actual contours array
    return cnts
class ShapeDetector:
    def __init__(self):
        pass
    def detect(self, c):
		# initialize the shape name and approximate the contour
        shape = "unidentified"
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        if len(approx) == 3:
            shape = "triangle"
        elif len(approx) == 4:
        	(x, y, w, h) = cv2.boundingRect(approx)
        	ar = w / float(h)
        	shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
        elif len(approx) == 5:
        	shape = "pentagon"
        else:
        	shape = "circle"
        return shape

cap = cv2.VideoCapture(0)
height = 720
width = 1280
maxArea = 0
while True:
    success, img = cap.read()
    resized = cv2.resize(img, (width, height))

    ratio = img.shape[0] / float(resized.shape[0])
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh_black = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY_INV)[1]
    thresh = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)[1]
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
    	cv2.CHAIN_APPROX_SIMPLE)
    cnts = grab_contours(cnts)
    sd = ShapeDetector()
    # Whiteboard loop
    for c in cnts:
        area = cv2.contourArea(c)
        if area < 99000:
            continue
        M = cv2.moments(c)
        if M["m00"] == 0:
            M["m00"] = .01
        if M["m00"] == 0:
            M["m00"] = .01
        cX = int((M["m10"] / M["m00"]) * ratio)
        cY = int((M["m01"] / M["m00"]) * ratio)
        shape = sd.detect(c)
        if shape !='rectangle':
            continue
        if shape == 'rectangle':
            # print(area)
            shape = 'whiteboard'
        c = c.astype("float")
        c *= ratio
        c = c.astype("int")
        cv2.drawContours(img, [c], -1, (0, 255, 0), 2)
        cv2.putText(img, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
    		0.5, (255, 255, 255), 2)

    #squares loop
    cnts = cv2.findContours(thresh_black.copy(), cv2.RETR_EXTERNAL,
    	cv2.CHAIN_APPROX_SIMPLE)
    cnts = grab_contours(cnts)
    for c in cnts:
        area = cv2.contourArea(c)  # Calculates area of each shape detected
        if area > 900000 or area <100:
            continue
        M = cv2.moments(c)
        if M["m00"] == 0:
            M["m00"] = .01
        if M["m00"] == 0:
            M["m00"] = .01
        cX = int((M["m10"] / M["m00"]) * ratio)
        cY = int((M["m01"] / M["m00"]) * ratio)
        shape = sd.detect(c)
        if shape !='square' and shape !='rectangle':
            continue
        if shape =='square':
            print(cX, cY)
        if shape =='rectangle':
            print(cX, cY)
        cv2.drawContours(img, [c], -1, (0, 255, 0), 2)
        cv2.putText(img, shape + " " + '(' + str(cX)+','+ str(cY) + ')', (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
    		0.5, (255, 255, 255), 2)

    cv2.imshow('object detection', cv2.resize(img, (width, height)))
    if cv2.waitKey(25) & 0xFF == ord('q'):
        cap.release()
        videoFile.release()
        cv2.destroyAllWindows()
        break
