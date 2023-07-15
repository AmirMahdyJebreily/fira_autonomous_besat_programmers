#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2	
import numpy as np

bridge = CvBridge()

def callback(data):

	speed = 4

	global velocity_publisher
	global vel_msg
	global nolineRec

	frame = bridge.imgmsg_to_cv2(data, "bgr8")

	image = frame	

	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	blur = cv2.GaussianBlur(gray, (3,3), 0)

	edge = cv2.Canny(blur, 50, 100)
	

	height, width = edge.shape	

	i = 80

	j = 60

	k = 100

	poly = np.array([
		[
			(10, frame.shape[0] - k),
			(int(frame.shape[1] / 2) - i, int(frame.shape[0] / 2) + j),
			(int(frame.shape[1] / 2) + i , int(frame.shape[0] / 2) + j),
			(frame.shape[1], frame.shape[0] - k)
		]
	])


	blank = np.zeros_like(image)		

	# mask = cv2.fillPoly(np.zeros_like(image), pts=[poly], color=255)

	mask = np.zeros_like(edge)	

	mask = cv2.fillPoly(mask, pts=[poly], color=255)

	maskedimg = cv2.bitwise_and(edge, mask)

	graymaskedimg = cv2.bitwise_and(gray, mask)

	lines = cv2.HoughLinesP(maskedimg, rho=3, theta=np.pi/45, threshold=10, lines=np.array([]), minLineLength=40, maxLineGap=5)


	final = image.copy()
	
	rawLines = image.copy()

	color = (0,255,0)

	error = 0

	if lines is not None:
		if (len(lines) == 1):
			lines = [lines]

		avglines, error = avg_line(blank, lines)

		if abs(error) > 1:
			print("speed reduced")
			speed = 1
		
		final = draw_lines(final, avglines,color)

		rawLines = draw_lines(rawLines, lines , (255,0,0))

		error = error 	
	else:
		speed = speed / 10
		final = draw_texts(final,"no line detected")

	steering = translate(-1.0,1.0,-3.0,3.0,float(error))

	steering * 0.2

	print(error, steering)

	vel_msg.linear.x = speed 

	vel_msg.angular.z = steering

	velocity_publisher.publish(vel_msg)

	# # print(error, steering)

	
	cv2.imshow("MASK",mask)
	cv2.imshow("IMAGE LINES", rawLines)
	cv2.imshow("GRAY MASKED IMAGE", graymaskedimg)
	cv2.imshow("FINAL", final)
	cv2.waitKey(10)

def receive():
	global nolineRec
	nolineRec = 0
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
	
def translate(inp_min, inp_max, out_min, out_max,inp_error):
	return float(inp_error - inp_min) * float(out_max-out_min) / float(inp_max - inp_min) + float(out_min)

def draw_lines(image, lines, color):
	#line_image = image.copy()
	for line in lines:
	# print(lines[0])
		x1, y1, x2, y2 = line.reshape(4) 
		# print(type(x1))
		cv2.line(image, (x1, y1) ,(x2, y2), color, 10)			
	return image

def draw_texts(image, text):
	font = cv2.FONT_HERSHEY_SIMPLEX
	fontScale = 3.0
	color = (0, 0, 255)
	thickness = 2
	return cv2.putText(image, text, (0, 400), font, fontScale, color, thickness)

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

	# print(left_line_avg_fix)
	# print(right_line_avg_fix)
	
	error = (left_line_avg_fix[0] + right_line_avg_fix[0]) / 2
		
	# print(error, out)
 
	return np.array([left_line, right_line]), error
    

if __name__ == "__main__":
    rospy.init_node("receiveImage" , anonymous=True)
    try:
        receive()
    except rospy.ROSInterruptException: pass