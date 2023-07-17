#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2	
import numpy as np


bridge = CvBridge()

def callback(data):
	global velocity_publisher
	global vel_msg
	
	frame = bridge.imgmsg_to_cv2(data, "bgr8")

	image = frame	

	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	blur = cv2.GaussianBlur(gray, (5,5), 0)

	edge = cv2.Canny(image, 50, 100)

	i = 1

	j = 20

	k = 100


	poly = np.array([
		[
			(10, frame.shape[0] - k),
			(int(frame.shape[1] / 2) - i, int(frame.shape[0] / 2) + j),
			(int(frame.shape[1] / 2) + i, int(frame.shape[0] / 2) + j),
			(frame.shape[1], frame.shape[0] - k)
		]
	])



	mask = np.zeros_like(edge)	

	mask = cv2.fillPoly(mask, poly, color=255)


	maskedimg = cv2.bitwise_and(edge, mask)

	lines = cv2.HoughLinesP(maskedimg, rho=3, theta=np.pi/45, threshold=20, lines=np.array([]), minLineLength=40, maxLineGap=5)

	print(lines)

	blank = np.zeros_like(image)	
	
	# avglines = avg_line(blank, lines)

	


	avglines, erorr = avg_line(blank, lines)
	

	vel_msg.angular.z = erorr
    
	vel_msg.linear.x = 3
	
	# final = image
	final = draw_lines(image, avglines)
	
	velocity_publisher.publish(vel_msg)

	cv2.imshow("win", final)
	cv2.waitKey(10)

	
def receive():
    rospy.Subscriber("/catvehicle/camera_front/image_raw_front", Image, callback)
    global velocity_publisher
    global vel_msg

    velocity_publisher = rospy.Publisher('/catvehicle/cmd_vel_safe', Twist, queue_size=10)
    vel_msg = Twist()

    # Init Twist values
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    
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
	if(isinstance(line, np.ndarray)):

		slope, intercept = line
		y1 = image.shape[0]
		y2 = int(y1 * (3/5))
		x1 = int((y1 - intercept) / slope)
		x2 = int((y2 - intercept) / slope)
		return np.array([x1, y1, x2, y2])
	else:
		return np.array([0,0,0,0])

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
	
	left_line_avg_fix = left_line_avg if (isinstance(left_line_avg, np.ndarray)) else np.array([0, 0]) 	

	
	right_line_avg_fix = right_line_avg if (isinstance(right_line_avg, np.ndarray)) else np.array([0, 0]) 	

	print(type(left_line_avg_fix), left_line_avg_fix)
	print(type(right_line_avg_fix), right_line_avg_fix)
	
	error = (left_line_avg_fix[0] + right_line_avg_fix[0]) / 2
	
	out = translate(float(error), -1.0, 1.0, -3.0, 3.0)
	
	print(error, out)
 
	return np.array([left_line, right_line]), out
    

def translate(x, in_min, in_max, out_min, out_max):
	return float(x - in_min) * float(out_max - out_min) / float(in_max - in_min) + float(out_min)



if __name__ == "__main__":
    rospy.init_node("receiveImage" , anonymous=True)
    try:
        receive()
    except rospy.ROSInterruptException: pass


