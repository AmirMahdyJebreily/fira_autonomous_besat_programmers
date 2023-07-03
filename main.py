# !/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

bridge = CvBridge()

def callback(data):
    frame = bridge.imgmsg_to_cv2(data, "bgr8")
    
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # blur = cv2.GaussianBlur(frame, (5,5), 0)
    edge = cv2.Canny(frame, 50, 100)
    t = 1
    j = 20
    low_h = 100
    poly = np.array([
        [
            (10,frame.shape[0] - low_h) ,
            (int(frame.shape[1] / 2) - t, int(frame.shape[0] / 2) + j),
            (int(frame.shape[1] / 2) + t, int(frame.shape[0] / 2) + j),
            (frame.shape[1], frame.shape[0] - low_h)
            ]
        ])
    print(frame.shape)
    
    mask = np.zeros_like(edge)

    mask = cv2.fillPoly(mask, poly , 255)
    mask = cv2.bitwise_and(edge, mask)
    lines = cv2.HoughLinesP(mask, rho=3, theta=np.pi/45, threshold=20, lines=np.array([]), minLineLength=40, maxLineGap=5)

    copy = frame.copy()
    lines = avg_line(frame, lines)
    final = draw_lines(copy, lines)
    cv2.imshow("final", final)
    cv2.waitKey(10)
    print(lines)

    def draw_lines(frame, lines):
    # line_image = np.zeros_like(image)
        line_image = frame.copy()
        print("lines",lines)
        for x1, y1, x2, y2 in lines:
            cv2.line(line_image, (x1, y1) ,(x2, y2), (0, 255, 0), 10)

        return line_image

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
    
        return np.array([left_line, right_line])
    
    
 
    

def receive():
    rospy.Subscriber("/catvehicle/camera_front/image_raw_front", Image, callback)
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("receiveImage" , anonymous=True)
    try:
        receive()
    except rospy.ROSInterruptException: pass