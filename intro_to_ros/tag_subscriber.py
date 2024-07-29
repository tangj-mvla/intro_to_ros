#!/usr/bin/env python3

import cv2
import matplotlib.pyplot as plt
from dt_apriltags import Detector
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
from cv_bridge import CvBridge
from std_msgs.msg import Int16
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
import numpy as np

class TagSubscriber(Node):
    def __init__(self):
        super().__init__("TagSubscriber")
        
        self.apriltag_subscriber = self.create_subscription(
            Image, 
            "bluerov2/camera", 
            self.apriltagCallback, 
            10 
        )

    def apriltagCallback(self, msg): 
        self.cvb = CvBridge()
        msg = self.cvb.cv2_to_imgmsg(, encoding="bgr8")

    def process_frame(self,frame):
        """
        Applies Gaussian blur and converts the frame to grayscale.

        Args:
            frame (numpy.ndarray): The input frame in BGR format.

        Returns:
            tuple: A tuple containing the Gaussian blurred frame and the grayscale frame.
        """
        gblur = cv2.GaussianBlur(frame, (5, 5), 0)
        gray = cv2.cvtColor(gblur, cv2.COLOR_BGR2GRAY)
        return gblur, gray

    def display_frame(self,frame, title='Frame'):
        """
        Displays the given frame using matplotlib.

        Args:
            frame (numpy.ndarray): The frame to be displayed.
            title (str): The title of the frame window.
        """
        plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        plt.title(title)
        plt.axis('off')
        plt.show()
        plt.pause(0.001)  # Pause to allow the image to be displayed

    def detect_april_tags(self,frame):
        """
        Detects AprilTags in the given frame and draws lines and IDs on the detected tags.

        Args:
            frame (numpy.ndarray): The input frame in grayscale format.

        Returns:
            numpy.ndarray: The frame with detected AprilTags drawn.
        """
        # Initialize the AprilTag detector
        at_detector = Detector(families='tag36h11',
                            nthreads=1,
                            quad_decimate=1.0,
                            quad_sigma=0.0,
                            refine_edges=1,
                            decode_sharpening=0.25,
                            debug=0)

        tags = at_detector.detect(frame, estimate_tag_pose=False, camera_params=None, tag_size=None)
        color_frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)

        for tag in tags:
            for idx in range(len(tag.corners)):
                cv2.line(color_frame, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))

            cv2.putText(color_frame, str(tag.tag_id),
                        org=(tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.8,
                        color=(0, 0, 255))
        
        return color_frame

    def process_video_iterative(self,video_path):
        """
        Processes a video, displaying and detecting AprilTags every 1000 frames.

        Args:
            video_path (str): The path to the video file.
        """
        cap = cv2.VideoCapture(video_path)
        frame_count = 0

        while cap.isOpened():
            isTrue, frame = cap.read()
            if not isTrue:
                break
            
            frame_count += 1
            
            if frame_count % 1000 == 0:
                gblur, gray = self.process_frame(frame)
                detection = self.detect_april_tags(gray)
                self.display_frame(detection, title='Grayscale Frame')
            
        cap.release()
        plt.close()

    # Run the function to process the video
    process_video_iterative('pool_video.mp4')