# This node is used to detect multiple objects through camera using Cascade and SVM
# It is similar to the cascade.py. The only changes are the type of msg and member of class,
# to store multiple objects' position
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose


class CascadeClassifier(Node):
    def __init__(self):
        super().__init__('cascade')
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

        self.br = CvBridge()
        
        # self.box_detector = cv2.CascadeClassifier('/home/nguyen/python_test/cascade/model3008/cascade.xml')
        self.box_detector = cv2.CascadeClassifier('/home/nguyen/ws_astorino/src/astorino_detect/astorino_detect/cascade_box.xml')
        # self.box_detector = cv2.CascadeClassifier('/home/nguyen/python_test/cascade/model_2020_0410_2/cascade.xml')
        # self.box_detector = cv2.CascadeClassifier('/home/nguyen/python_test/cascade_round/model_test2/cascade.xml')


        self.box_detected_publisher = self.create_publisher(Image, 'detected', 10)
        self.hough_publisher = self.create_publisher(Image, 'hough', 10)


        self.u_pixel = []
        self.v_pixel = []
        self.z = []
        self.theta = []

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_publisher)
        self.point_publisher_ = self.create_publisher(PoseArray, 'pose_stamped', 10)



        window_size = (24,24)
        block_size = (8,8)
        block_stride = (4,4)
        cell_size = (4,4)
        num_bins = 16

        # window_size = (21,14)
        # block_size = (6,4)
        # block_stride = (3,2)
        # cell_size = (3,2)

        
        self.hog = cv2.HOGDescriptor(window_size, block_size, block_stride, cell_size, num_bins)
        self.svm = cv2.ml.SVM_load("/home/nguyen/ws_astorino/src/astorino_detect/astorino_detect/svm_box.xml")
        # self.svm = cv2.ml.SVM_load("/home/nguyen/python_test/cascade_round/test_svm.xml")



    def image_callback(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        boxes = self.box_detector.detectMultiScale(frame_gray)
        self.u_pixel.clear()
        self.v_pixel.clear()
        self.theta.clear()

        for (x,y,w,h) in boxes:
            sub_frame = frame_gray[y:y+h,x:x+w]
            check_hog = self.check_svm(sub_frame)

            if (check_hog == 1):
                frame = cv2.rectangle(frame, (x,y), (x + w, y + h), (0, 0, 255), 1)
                self.u_pixel.append(x + int(w/2))
                self.v_pixel.append(y + int(h/2))
                self.theta.append(self.orientation_hough_link(sub_frame))


        
        self.box_detected_publisher.publish(self.br.cv2_to_imgmsg(frame, encoding='rgb8'))

    def orientation_hough_link(self, frame):
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
                    theta.append(0)
                    cv2.line(cdst, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv2.LINE_AA)
        self.hough_publisher.publish(self.br.cv2_to_imgmsg(cdst, encoding='bgr8'))
        if theta is not None:
            return np.max(theta)
        else:
            return 0

    def depth_callback(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        self.z.clear()
        for i in range(len(self.v_pixel)):
            self.z.append(frame[self.v_pixel[i], self.u_pixel[i]])

    def check_svm(self, frame):
        img = cv2.resize(frame, (24,24))
        hog_img = self.hog.compute(img)
        input = np.matrix(hog_img, dtype=np.float32)
        point = self.svm.predict(input)[1]
        return point


    def timer_publisher(self):
        point = PoseArray()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = "camera_color_optical_frame"
        max_len = min(len(self.v_pixel), len(self.u_pixel), len(self.z))
        for i in range(max_len):
            new_pose = Pose()
            new_pose.position.z = float(self.z[i]) / 1000
            new_pose.position.y = (self.v_pixel[i] - 365.14520263671875) * new_pose.position.z / 638.1966552734375
            new_pose.position.x = (self.u_pixel[i] - 642.7182006835938) * new_pose.position.z / 638.6309204101562

            new_pose.orientation.x = 0.0
            new_pose.orientation.y = np.sin(np.deg2rad(self.theta[i])/2)
            new_pose.orientation.z = 0.0
            new_pose.orientation.w = np.cos(np.deg2rad(self.theta[i])/2)

            point.poses.append(new_pose)

        self.point_publisher_.publish(point)



def main(args=None):
    rclpy.init(args=args)
    cascade = CascadeClassifier()
    rclpy.spin(cascade)
    cascade.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()