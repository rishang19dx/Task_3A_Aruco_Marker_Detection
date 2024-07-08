#!/usr/bin/env python3

import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ArucoDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        print("ArucoDetector initialized and subscribed to /camera/rgb/image_raw")

    def image_callback(self, data):
        print("Image received")
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            print(f"Markers detected: {ids}")
            aruco.drawDetectedMarkers(cv_image, corners, ids)
            for i in range(len(ids)):
                print(f"ID: {ids[i]} - Corners: {corners[i]}")

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('aruco_detector', anonymous=True)
    print("Aruco detector node started")
    aruco_detector = ArucoDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
