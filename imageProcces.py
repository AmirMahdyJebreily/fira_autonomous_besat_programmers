#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2	
import numpy as np


bridge = CvBridge()

def callback(data):
	frame = bridge.imgmsg_to_cv2(data, "bgr8")
	'''
	TODO:Implementation
	'''
	image = frame	

	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	blur = cv2.GaussianBlur(gray, (5,5), 0)

	edge = cv2.Canny(image, 50, 100)


	height, width = edge.shape	

	i = 200

	j = -100

	k = 125	

	poly = np.array([[(0, height - k), (width/2 - i ,height/2 + j) , (width/2 + i , height/2 + j),( width, height - k)]], np.int32)

	# print(poly)

	mask = np.zeros_like(edge)	

	mask = cv2.fillPoly(mask, pts=[poly], color=255)


	maskedimg = cv2.bitwise_and(edge, mask)

	lines = cv2.HoughLinesP(maskedimg, rho=6, theta=np.pi/60, threshold=100, lines=np.array([]), minLineLength=40, maxLineGap=20)

	print(lines)

	blank = np.zeros_like(image)	

	avglines , error = avg_line(blank, lines)
	
	final = draw_lines(image, avglines)

	print(translate(-1,1, -3, 3, error))

	cv2.imshow("win", final)
	cv2.waitKey(10)
def translate(inp_min, inp_max, out_min, out_max,inp_error):
	return ((((inp_error - inp_min ) * (out_max - out_min)) / (inp_max - inp_min))+ out_min)
def receive():
    rospy.Subscriber("/catvehicle/camera_front/image_raw_front", Image, callback)
    rospy.spin()
def draw_lines(image, lines):
	line_image = np.zeros_like(image)
	#line_image = image.copy()
	print(lines)
	# for line in lines:
	for x1, y1, x2, y2 in lines:
		cv2.line(image, (x1, y1) ,(x2, y2), (0, 255, 0), 10)
			
	return image

def make_coordinates(image, line):
	slope, intercept = line
	y1 = image.shape[0]
	y2 = int(y1 * (3/5))
	x1 = int((y1 - intercept) / slope)
	x2 = int((y2 - intercept) / slope)
	return np.array([x1, y1, x2, y2])

def avg_line(image, lines):
	left_lines = []
	right_lines = []
	for line in lines:
		x1, y1, x2, y2 = line.reshape(4) 
		parameters = np.polyfit((x1,x2), (y1,y2), 1)
		slope = parameters[0]
		intercept = parameters[1]
		if(slope <= 0):
			if(slope != 0):
				left_lines.append((slope, intercept))
		else:
			right_lines.append((slope, intercept))

	left_line_avg = np.average(left_lines, axis=0) 
	right_line_avg = np.average(right_lines, axis=0) 


	left_line = make_coordinates(image, left_line_avg)
	right_line = make_coordinates(image, right_line_avg)
 
	error = (left_line_avg[0] + right_line_avg[0]) / 2
	print(left_line_avg[0], right_line_avg[0])
	print(error)
 
	return np.array([left_line, right_line]) , error
if __name__ == "__main__":
    rospy.init_node("receiveImage" , anonymous=True)
    try:
        receive()
    except rospy.ROSInterruptException: pass
