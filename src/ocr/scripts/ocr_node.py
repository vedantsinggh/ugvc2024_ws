#!/usr/bin/env python3 

import rospy 
import cv2 
import pytesseract 
from pytesseract import Output 
from sensor_msgs.msg import Image
from std_msgs.msg import String 
from cv_bridge import CvBridge, CvBridgeError

class OCRNode:
    def __init__(self):
        rospy.init_node('ocr_node', anonymous=True)

        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

        # Publishers
        self.digits_pub = rospy.Publisher('/recognized_digits', String, queue_size=10)

    def preprocess_image(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY_INV)

        return binary 

    def recognize_digits(self, image):
        custom_config = r'--oem 3 --psm 6 outputbase digits'
        data = pytesseract.image_to_data(image, config=custom_config, output_type=Output.DICT)

        digits = []
        for i in range(len(data['text'])):
            if data['text'][i].isdigit():
                digits.append(data['text'][i])
        
        return digits

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return 
        
        preprocessed_image = self.preprocess_image(cv_image)
        recognized_digits = self.recognize_digits(preprocessed_image)

        if recognized_digits:
            recognized_digits_str = ''.join(recognized_digits)
            rospy.loginfo(f"Recognized digits: {recognized_digits_str}")
            self.digits_pub.publish(recognized_digits_str)
        else:
            rospy.loginfo("No digits recognized")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = OCRNode()
        node.run()
    except rospy.ROSInterruptException:
        pass