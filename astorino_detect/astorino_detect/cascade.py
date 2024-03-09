# This node is used to detect one object through camera using Cascade and SVM
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped


class CascadeClassifier(Node):
    def __init__(self):
        super().__init__('cascade')
        # Create subscribers for getting information from a RGB camera and a depth camera
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
        self.image_subscription

        self.depth_subscription = self.create_subscription(
            Image,
            'camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        self.depth_subscription

        # Use for the conversion between OpenCV image and ROS Image message
        self.br = CvBridge()

        # Importing the model for Cascade
        self.box_detector = cv2.CascadeClassifier('/home/nguyen/python_test/cascade/model_2020_0410_2/cascade.xml')
        # self.box_detector = cv2.CascadeClassifier('/home/nguyen/python_test/cascade_round/model_test2/cascade.xml')

        # Create publisher for publish the image with bounding boxes covering objects
        self.box_detected_publisher = self.create_publisher(Image, 'detected', 10)
        self.hough_publisher = self.create_publisher(Image, 'hough', 10)

        # Create members for storing the position of a detected object in pixel coordinates
        self.u_pixel = 0
        self.v_pixel = 0
        self.z = 0
        self.theta = 0

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_publisher)
        # self.point_publisher_ = self.create_publisher(PointStamped, 'point_stamped', 10)
        self.point_publisher_ = self.create_publisher(PoseStamped, 'pose_stamped', 10)


        # Parameters for extracting HOG features. Please modify for detecting cylinders
        window_size = (24,24)
        block_size = (8,8)
        block_stride = (4,4)
        cell_size = (4,4)
        num_bins = 16
        # Below are the parameters for cylinders
        # window_size = (21,14)
        # block_size = (6,4)
        # block_stride = (3,2)
        # cell_size = (3,2)

        # Create HOG instance for extracting HOG features
        self.hog = cv2.HOGDescriptor(window_size, block_size, block_stride, cell_size, num_bins)
        
        # Load the SVM model, change the location of the .xml file when using
        self.svm = cv2.ml.SVM_load("/home/nguyen/python_test/cascade/test_svm2410.xml")
        # self.svm = cv2.ml.SVM_load("/home/nguyen/python_test/cascade_round/test_svm.xml")



    def image_callback(self, msg):
        # Subscribe, get the image msg, convert to cv2 
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # Detect objects using Cascade
        boxes = self.box_detector.detectMultiScale(frame_gray)

        for (x,y,w,h) in boxes:
            # Double check by using SVM with HOG
            sub_frame = frame_gray[y:y+h,x:x+w]
            check_hog = self.check_svm(sub_frame)

            if (check_hog == 1):
                # Draw bounding boxes around the objects which can pass the SVM
                frame = cv2.rectangle(frame, (x,y), (x + w, y + h), (0, 0, 255), 1)

                # Store the pixel coordinates of the center of bounding box
                self.u_pixel = x + int(w/2)
                self.v_pixel = y + int(h/2)
        
        # Publish the image with bounding boxes covering detected objects
        self.box_detected_publisher.publish(self.br.cv2_to_imgmsg(frame, encoding='rgb8'))

    def orientation_hough_link(self, frame):
        # Using hough transform to extract the lines in the bottom of object to estimate the orientation
        # But it is not stable yet
        gray = frame
        dst = cv2.Canny(gray, 100, 200, None, 3)
        cdst = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 15, None, 5, 10)
        theta = [0]
        if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                if (l[1] > dst.shape[1] // 2 and l[3] > dst.shape[1] // 2):
                    theta_temp = np.arctan2(l[3] - l[1], l[2] - l[0])
                    theta_temp = np.round(np.rad2deg(theta_temp))
                    theta_temp %= 45
                    theta.append(theta_temp)
                    cv2.line(cdst, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv2.LINE_AA)

        self.hough_publisher.publish(self.br.cv2_to_imgmsg(cdst, encoding='bgr8'))

        if theta is not None:
            return np.max(theta)
        else:
            return 0

    def depth_callback(self, msg):
        # Get the depth information at the stored pixel coordinates and save it
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        self.z = frame[self.v_pixel, self.u_pixel]

    def check_svm(self, frame):
        # Compute HOG and pass to SVM
        img = cv2.resize(frame, (24,24))
        hog_img = self.hog.compute(img)
        input = np.matrix(hog_img, dtype=np.float32)
        point = self.svm.predict(input)[1]
        return point

    def timer_publisher(self):
        # Base on the pixel coordinates and the depth information, calculate
        # the position of the detected object in the camera_frame
        point = PoseStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = "camera_color_optical_frame"

        # The constant parameters here are the paramters of the used camera
        # It can be directly taken from the camera_info topic
        point.pose.position.z = float(self.z) / 1000
        point.pose.position.y = (self.v_pixel - 365.14520263671875) * point.pose.position.z / 638.1966552734375
        point.pose.position.x = (self.u_pixel - 642.7182006835938) * point.pose.position.z / 638.6309204101562

        # Convert the orientation to quaternion
        point.pose.orientation.x = 0.0
        point.pose.orientation.y = np.sin(np.deg2rad(self.theta)/2)
        point.pose.orientation.z = 0.0
        point.pose.orientation.w = np.cos(np.deg2rad(self.theta)/2)
        self.point_publisher_.publish(point)



def main(args=None):
    rclpy.init(args=args)
    cascade = CascadeClassifier()
    rclpy.spin(cascade)
    cascade.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()