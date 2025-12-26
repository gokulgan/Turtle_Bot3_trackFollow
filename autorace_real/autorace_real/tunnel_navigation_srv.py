

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np

from sensor_msgs.msg import Image, LaserScan
from example_interfaces.srv import Trigger
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

class TunnelDetectorService(Node):
    def __init__(self):
        super().__init__('tunnel_detector_srv')

        self.bridge = CvBridge()
        self.latest_image = None

        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_pub)
        self.callback_group = ReentrantCallbackGroup()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_sub,
            callback_group=self.callback_group
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_sub,
            callback_group=self.callback_group
        )

        self.srv = self.create_service(
            Trigger,
            'detect_tunnel_sign',
            self.detect_tunnel_sign_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('TunnelDetectorService node started.')

        self.is_driving_toward_wall = False
        # self.wall_distance_threshold = 0.3  # meters

        self.timer = self.create_timer(
            0.1,  # 10 Hz
            self.timer_callback,
            callback_group=self.callback_group
        )

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            self.latest_image = None
            return

        cv2.imshow("Camera View", self.latest_image)
        cv2.waitKey(1)

    def detect_tunnel_sign_callback(self, request, response):
        if self.latest_image is None:
            response.success = False
            response.message = 'No image received yet.'
            return response

        detected, debug_info = self.detect_sign(self.latest_image)

        if detected:
            self.move_forward()

        response.success = detected
        response.message = debug_info
        return response

    def detect_sign(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])

        red_mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower_red1, upper_red1),
            cv2.inRange(hsv, lower_red2, upper_red2)
        )
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        contours_red, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_yellow, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        debug_message = f"Red contours: {len(contours_red)}, Yellow contours: {len(contours_yellow)}."

        if not contours_red or not contours_yellow:
            return False, f"No sign detected. {debug_message}"

        for rc in contours_red:
            if len(cv2.approxPolyDP(rc, 0.04 * cv2.arcLength(rc, True), True)) != 3:
                continue

            for yc in contours_yellow:
                M = cv2.moments(yc)
                if M["m00"] == 0:
                    continue
                cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                if cv2.pointPolygonTest(rc, (cx, cy), False) >= 0:
                    self.get_logger().info(f"Tunnel sign detected! {debug_message}")
                    return True, f"Tunnel sign detected. {debug_message}"

        return False, f"No sign detected. {debug_message}"

    def move_forward(self):
        self.is_driving_toward_wall = True
        self.get_logger().info('Started driving forward.')

    def timer_callback(self):
        if self.is_driving_toward_wall:
            msg = Twist()
            msg.linear.x = 0.1
            self.cmd_vel_pub.publish(msg)

    def scan_callback(self, msg):
        right = msg.ranges[270]
        if not self.is_driving_toward_wall:
            return

        index = min(200, len(msg.ranges) - 1)
        distance = msg.ranges[index]

        self.get_logger().info(f"right == {right}")
        # if np.isinf(distance) or np.isnan(distance):
        #     distance = 0.0

        # if distance != 0.0 and distance < self.wall_distance_threshold:
        #     self.stop_robot()
        if right !=0 and right < 0.3 :
            self.stop_robot()


    def stop_robot(self):
        self.is_driving_toward_wall = False
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().info('Wall detected on the right. Stopped.')

def main(args=None):
    rclpy.init(args=args)
    node = TunnelDetectorService()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
