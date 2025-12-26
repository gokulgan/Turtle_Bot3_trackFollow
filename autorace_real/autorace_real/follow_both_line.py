import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, qos_profile)
        self.br = CvBridge()
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Velocity gains
        self.linear_velocity_gain = 0.1
        self.angular_velocity_gain = 0.04

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        
        # Convert ROS image to OpenCV image
        img = self.br.imgmsg_to_cv2(data, 'bgr8')
        frame = cv2.resize(img, (320, 240))
        h, w = frame.shape[:2]
        roi = frame[int(h*0.4):, :]  # Region of interest

        # Convert BGR to HSV
        imgHSV = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Yellow mask
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        mask_yellow = cv2.inRange(imgHSV, lower_yellow, upper_yellow)

        # White mask
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 25, 255])
        mask_white = cv2.inRange(imgHSV, lower_white, upper_white)

        # Visualization
        combined_mask = cv2.bitwise_or(mask_white, mask_yellow)
        cv2.imshow("Combined Mask (Yellow + White)", combined_mask)
        cv2.waitKey(1)

        # Centroids
        cx_y = self.find_bottom_centroid(mask_yellow)
        cx_w = self.find_bottom_centroid(mask_white)

        img_center = roi.shape[1] // 2
        twist = Twist()

        if cx_y is not None and cx_w is not None:
            # Both lines → follow center
            lane_center = (cx_y + cx_w) // 2
            error = img_center - lane_center
            self.get_logger().info(f'[FOLLOW] Lane center: {lane_center}, error: {error}')

            twist.linear.x = self.linear_velocity_gain
            twist.angular.z = self.angular_velocity_gain * error / 100.0

        elif cx_y is not None:
            # Only yellow → spin right slowly to find white
            self.get_logger().warn('White line lost! Rotating right to find it.')
            twist.linear.x = 0.0
            twist.angular.z = -0.2

        elif cx_w is not None:
            # Only white → spin left slowly to find yellow
            self.get_logger().warn('Yellow line lost! Rotating left to find it.')
            twist.linear.x = 0.0
            twist.angular.z = 0.2

        else:
            # Neither detected
            self.get_logger().warn('No lines detected! Stopping.')
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher_.publish(twist)

    def find_bottom_centroid(self, mask):
        ys, xs = np.where(mask > 0)
        if len(ys) > 0:
            max_y = np.max(ys)
            bottom_x = xs[ys == max_y]
            return int(np.mean(bottom_x))
        else:
            return None

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
