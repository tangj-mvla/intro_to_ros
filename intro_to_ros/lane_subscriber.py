#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Int16
from cv_bridge import CvBridge
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
import numpy as np

class LaneSubscriber(Node):

    image = None
    heading = None

    def __init__(self):
        super().__init__("LaneSubscriber")

        self.cvb = CvBridge()

        self.heading_subscriber = self.create_subscription(
            Int16,
            "bluerov2/heading",
            self.headingSubscriberCallback,
            10
        )

        self.image_subscriber = self.create_subscription(
            Image,
            "bluerov2/camera",
            self.imageCallback,
            10
        )

        self.desired_heading_publisher = self.create_publisher(
            Int16,
            "bluerov2/desired_heading",
            10
        )

    def headingSubscriberCallback(self,msg):
        self.heading = msg.data
        self.rotation()
    
    def imageCallback(self,msg):
        image = self.cvb.imgmsg_to_cv2(msg)
        self.image = image
        self.rotation()

    def rotation(self):
        if (self.image == None or self.heading == None):
            return
        img = self.image.data
        lines = self.detect_lines(img)
        lanes = self.detect_lanes(lines)
        middleLanes = self.get_lane_center(lanes)
        recommendation = self.recommendation(middleLanes)
        if (recommendation == "left"):
            self.desired_heading_publisher.publish(self.heading-10)
        elif (recommendation == "right"):
            self.desired_heading_publisher.publish(self.heading+10)

    def detect_lines(self,img, threshold1 = 10, threshold2 = 20, apertureSize = 3, minLineLength = 800, maxLineGap = 100):
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, threshold1 = threshold1, threshold2 = threshold2, apertureSize = apertureSize) # detect edges
        lines = cv2.HoughLinesP(
                        edges,
                        1,
                        np.pi/180,
                        100,
                        minLineLength = minLineLength,
                        maxLineGap = maxLineGap,
                ) # detect lines
        return lines

    def get_slopes_intercepts(self,lines):
            slopes = []
            intercepts = []
            for line in lines:
                x1,y1,x2,y2 = line[0]
                slope = (y2-y1)/(x2-x1)
                inverse_slope = (x2-x1)/(y2-y1)
                intercept = x1 - y1*inverse_slope
                slopes.append(slope)
                intercepts.append(intercept)
            return slopes, intercepts
        
    def detect_lanes(self,lines):
        # group lines based on slope
        slopes,intercepts = self.get_slopes_intercepts(lines)
        linesGroup = []
        for i in range(len(lines)):
            if (i == 0):
                linesGroup.append([lines[i]])
            else:
                slope = slopes[i]
                added = False 
                for group in linesGroup:
                    x1,y1,x2,y2 = group[0][0]
                    slope2 = (y2-y1)/(x2-x1)
                    
                    if (np.abs(1.0-slope/slope2) < 0.05): # error within 5%
                        group.append(lines[i])
                        added = True
                if (not added):
                    linesGroup.append([lines[i]])
        
        # create singular line from a group of lines
        lessLines = []
        for group in linesGroup:
            x1,y1,x2,y2 = float('inf'),float('inf'),float('-inf'),float('-inf')
            for line in group:
                l = line[0]
                if (l[0] < x1 and l[1] < y1):
                    x1,y1 = l[0],l[1]
                if (l[2] > x2 and l[3] > y2):
                    x2,y2 = l[2],l[3]
            lessLines.append(np.array([[x1,y1,x2,y2]]))
        
        # sort based on slope
        lanes = []
        slopes, intercepts = self.get_slopes_intercepts(lessLines)
        slopesArgs = np.argsort(slopes)

        # need to change so sorted range goes from barely neg to max neg to max pos to barely pos
        i = 0
        while (i < len(slopesArgs)):
            if (slopes[slopesArgs[i]] > 0):
                slice = slopesArgs[0:i]
                slice = slice[::-1]
                slopesArgs[0:i] = slice
                break
            i += 1
        
        i = len(slopesArgs)-1
        while (i >= 0):
            if (slopes[slopesArgs[i]] < 0):
                slice = slopesArgs[i+1:len(slopesArgs)-1]
                slice = slice[::-1]
                slopesArgs[i+1:len(slopesArgs)-1] = slice
                break
            i -= 1
            
        sortedSlopes = []
        sortedLines = []
        for i in range(len(lessLines)):
            sortedLines.append(lessLines[slopesArgs[i]])
            sortedSlopes.append(slopes[slopesArgs[i]])

        # finding lanes
        slopes = sortedSlopes
        i = 1
        while i < len(sortedLines)-1:
            line = sortedLines[i]
            slope = slopes[i]
            x1,y1,x2,y2 = line[0]
            intercept = (2000-y1)/slope + x1

            x1,y1,x2,y2 = sortedLines[i-1][0]
            prevIntercept = (2000-y1)/slope + x1

            x1,y1,x2,y2 = sortedLines[i+1][0]
            nextIntercept = (2000-y1)/slope + x1
            if (np.abs(1.0-slope/slopes[i-1]) < np.abs(1.0-slope/slopes[i+1]) and np.abs(intercept-prevIntercept)<np.abs(intercept-nextIntercept) and np.abs(intercept-prevIntercept) < 2000):
                lanes.append([sortedLines[i-1],sortedLines[i]])
            elif (np.abs(1.0-slope/slopes[i-1]) >= np.abs(1.0-slope/slopes[i+1]) and np.abs(intercept-prevIntercept)>=np.abs(intercept-nextIntercept) and np.abs(intercept-nextIntercept) < 2000):
                lanes.append([sortedLines[i],sortedLines[i+1]])
            i+=1

        return lanes

    def get_lane_center(self,lanes):
        middleLanes = []
        for lane in lanes:
            line1,line2 = lane[0], lane[1]
            x1, y1, x2, y2 = line1[0]
            line1x1,line1y1,line1x2,line1y2 = line1[0]
            line2x1,line2y1,line2x2,line2y2 = line2[0]
            # average points
            x1 = (line1x1 + line2x1)/2
            y1 = (line1y1 + line2y1)/2
            x2 = (line1x2 + line2x2)/2
            y2 = (line1y2 + line2y2)/2
            middleLanes.append(np.array([[x1,y1,x2,y2]]))
        return middleLanes
    
    def recommendation(self,middleLanes):
        intercept = float('inf')
        slope = float('inf')
        for line in middleLanes:
            x1,y1,x2,y2 = line[0]
            laneSlope = (y2-y1)/(x2-x1)
            laneIntercept = (2000-y1)/slope + x1
            if (np.abs(laneIntercept-3824/2) < np.abs(intercept-3824/2)):
                intercept = laneIntercept
                slope = laneSlope
        if (np.abs(intercept-3824/2) < 250):
            return "forward"
        if (slope < 0):
            return "left"
        return "right"
    
def main(args = None):
    rclpy.init(args = args)
    lane_subscriber = LaneSubscriber()

    try:
        rclpy.spin(lane_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        lane_subscriber.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()