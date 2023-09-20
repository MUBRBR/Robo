# This script shows how to open a camera the picamera2 module and grab frames and show these.
# Kim S. Pedersen, 2023

import cv2 # Import the OpenCV library
import time
import arcuco
import numpy as np

ARUCO_DICT = {
"DICT_6X6_250" : cv2.aruco.DICT_6X6_250
}

aruco_type = "DICT_6X6_250"
id = 1

arucoDict = cv2.aruco.Dictionary_get()


cv2.aruco.detectMarkers()
cv::Mat inputImage;
...
std::vector<int> markerIds;
std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
cv::aruco::ArucoDetector detector(dictionary, detectorParams);
detector.detectMarkers(inputImage, markerCorners, markerIds, rejectedCandidates);