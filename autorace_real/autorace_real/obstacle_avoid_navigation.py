




import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import cv2
import numpy as np
from cv_bridge import CvBridge
from autorace_interfaces.action import DetectWhiteLine  
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer

class SquareWallFollower(Node):
    def __init__(self):
        super().__init__('follow_wall_nodee')
        self._goal_active = False
        self.white_line_detected = False

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, qos_profile)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self._action_server = ActionServer(
            self,
            DetectWhiteLine,
            'detect_white_line',  # <-- This determines the action name
            self.execute_callback)
        # Parameters
        self.target_distance = 0.25

        self.forward_speed = 0.06
        self.turn_speed = 0.3 #normal  turn while alligning to wall
        self.TNturn_speed = 0.4#for turning corner having no right and front values while following wall
        self.Nturn_speed = 0.3#for adjusting to wall while runing forward.


        self.state = "approach_wall"  #initial_assignment_for wall_detection"

        
    def listener_callback(self, data):
        # Just store the image if we're in a state that needs camera processing
        self.latest_image = data
        self.image_ready = True
    async def execute_callback(self, goal_handle):
        self.get_logger().info('Goal received! Starting white line detection...')
        self._goal_active = True  # Enable processing
        feedback_msg = DetectWhiteLine.Feedback()
        
        while rclpy.ok() and self._goal_active:
            if self.white_line_detected:
                result = DetectWhiteLine.Result()
                result.success = True
                goal_handle.succeed(result)
                self._goal_active = False  # Reset
                return result
                
            feedback_msg.processing = True
            goal_handle.publish_feedback(feedback_msg)
            rate = self.create_rate(10)  # 10 Hz = 0.1 second delay
            rate.sleep()

        
        # If loop exits without detection
        result = DetectWhiteLine.Result()
        result.success = False
        goal_handle.abort()
        return result
    def scan_callback(self, msg):
        if not self._goal_active:  # Skip processing if no active goal
            return
        twist = Twist()
        if self.state == "camera_mode" and self.image_ready:
            try:
                self.get_logger().info("camera_mode")
                
                img = self.br.imgmsg_to_cv2(self.latest_image, 'bgr8')
                frame = cv2.resize(img, (320, 240))
                h, w = frame.shape[:2]
                roi = frame[int(h*0.4):, :]
                
                # Your image processing...
                imgHSV = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                lower_white = np.array([0, 0, 200])
                upper_white = np.array([180, 25, 255])
                lower_yellow = np.array([20, 100, 100])
                upper_yellow = np.array([35, 255, 255])
                mask_white = cv2.inRange(imgHSV, lower_white, upper_white)
                mask_yellow = cv2.inRange(imgHSV, lower_yellow, upper_yellow)
                
                # Center square detection
                mh, mw = mask_white.shape[:2]
                center_square = mask_white[int(mh/3):int(2*mh/3), int(mw/3):int(2*mw/3)]
                white_pixel_count = cv2.countNonZero(center_square)
                self.get_logger().info(f"NEW white count {white_pixel_count}")
                white_detected = cv2.countNonZero(mask_white) > 200
                yellow_detected = cv2.countNonZero(mask_yellow) > 200
                
                # if white_pixel_count > 10:
                self.get_logger().info("White line detected - stopping")
                twist.linear.x = 0.00
                twist.angular.z = 0.2
                if white_detected and yellow_detected:
                    self.get_logger().info("Both detected")
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                self.publisher.publish(twist)
                return
                    
            except Exception as e:
                self.get_logger().error(f"Camera error: {str(e)}")
                self.image_ready = False

        
        

        self.br = CvBridge()
        img = self.br.imgmsg_to_cv2(self.latest_image, 'bgr8')
        frame = cv2.resize(img, (320, 240))
        h, w = frame.shape[:2]
        roi = frame[int(h*0.4):, :]
        cv2.imshow("HSV Image", roi)
        # Convert BGR to HSVs
        imgHSV = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 25, 255])
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        mask_yellow = cv2.inRange(imgHSV, lower_yellow, upper_yellow)
        roi_yellow = mask_yellow
        mask_white = cv2.inRange(imgHSV, lower_white, upper_white)
        roi_white = mask_white
        # Get dimensions of mask
        mh, mw = mask_white.shape[:2]
        white_detected = cv2.countNonZero(mask_white) > 200
        yellow_detected = cv2.countNonZero(mask_yellow) > 200
   
        square_h_start = int(mh * 0.7)
        square_h_end = int(mh * 0.9)

        # Keep the same horizontal center
        square_w_start = int(mw / 3)
        square_w_end = int(2 * mw / 3)

        center_square = mask_white[square_h_start:square_h_end, square_w_start:square_w_end]
        cv2.imshow("Center Square", center_square)

        

    # Show the center square for debugging
        cv2.imshow("Center Square", center_square)
        cv2.waitKey(1)

    # You can now check if white is visible
        white_pixel_count = cv2.countNonZero(center_square)
        

        front = msg.ranges[0]
        right = msg.ranges[270]

        rightup=msg.ranges[280]#in betweeen front and right
        frontup=msg.ranges[260]#in between back and right
        chekright=msg.ranges[300]
        left=msg.ranges[90]
        
        self.get_logger().info(f"State: {self.state} | Front: {front:.2f}, Right: {right:.2f} | rightup: {rightup:.2f}, frontupght: {frontup:.2f}")
        # if self.state == "detect_wall":#detects wall
        #     if front<0.25:
        #         twist.linear.x=0.0
        #     else:
        #         self.state = "approach_wall"
        
        if self.state == "approach_wall":#approaching
            # Move forward until close to a wall
            if front > self.target_distance:
                twist.linear.x = self.forward_speed
            else:
                self.state = "align_to_wall"
                
        elif self.state == "align_to_wall":

            twist.linear.x = 0.00
            twist.angular.z = self.turn_speed#just turn with no forward speed to allign to wall
            self.get_logger().info(f"State: {self.state} | rightup: {rightup:.2f}, frontup: {frontup:.2f}")
            if right<=0.24 and right!=0.00:
                self.get_logger().info("triggered_to folow")
                
                self.state = "follow_wall"# When we detect the side wall
                
        elif self.state == "follow_wall":
            camera_white_detected = False
            if self.image_ready:
                try:
                    img = self.br.imgmsg_to_cv2(self.latest_image, 'bgr8')
                    frame = cv2.resize(img, (320, 240))
                    h, w = frame.shape[:2]
                    roi = frame[int(h*0.4):, :]
                    
                    # Image processing (same as before)
                    imgHSV = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                    lower_white = np.array([0, 0, 170])
                    upper_white = np.array([180, 25, 255])
                    mask_white = cv2.inRange(imgHSV, lower_white, upper_white)
                    
                    # Center square detection
                    mh, mw = mask_white.shape[:2]
                    square_h_start = int(mh * 0.7)
                    square_h_end = int(mh * 0.9)

                    # Keep the same horizontal center
                    square_w_start = int(mw / 3)
                    square_w_end = int(2 * mw / 3)
                    center_square = mask_white[square_h_start:square_h_end, square_w_start:square_w_end]
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# Define white in HSV: low saturation, high value


                    # Get white mask

                    white_pixel_count = cv2.countNonZero(center_square)
                    self.get_logger().info(f"white_pixel count = {white_pixel_count}")
                    # if white_pixel_count > 30:
                    #     camera_white_detected = True
                        
                except Exception as e:
                    self.get_logger().warn(f"Camera error: {str(e)}")
                    self.image_ready = False
            
            # PID control for wall following
            # self.get_logger().info("started_to folow")
            # error = right - self.target_distance
            # self.get_logger().info(f"follow right:{right}")
            twist.linear.x = 0.04
            
            # self.get_logger().info(f"error:{error}")
            if white_pixel_count > 40:
                 
                 twist.linear.x = 0.00
                 self.get_logger().info(f"NEW white count {white_pixel_count}")
                 self.white_line_detected = True
                 self.state="camera_mode"

            if chekright==0:
                 twist.linear.x = 0.00
                 twist.angular.z = -0.55
                 if chekright!=0.0:
                     self.state="follow_wall"


                 
        elif self.state == "turn_corner":
            # Execute 90° left turn
            self.get_logger().info(f"state={self.state}")
            twist.angular.z = -self.TNturn_speed
            twist.linear.x = 0.02
            if white_pixel_count > 30:
                self.get_logger().info(f"NEW white count {white_pixel_count}")
                self.state="camera_mode"
            # elif front<self.target_distance:
            #     self.state="align_to_wall"
            elif rightup!=0.00 or frontup!=0.00:
                self.state = "follow_wall"
                
        
        elif self.state == "tight corner":
            twist.angular.z = self.turn_speed
            self.get_logger().info(f"right:{right} tighttt corner")
            if white_pixel_count > 30:
                self.get_logger().info(f"NEW white count {white_pixel_count}")
                self.state="camera_mode"
            # When front clears and we detect new wall
            elif right<0.5 and front >0.7:
                self.state = "follow_wall"  


        self.publisher.publish(twist)



        
# def main(args=None):
#     rclpy.init(args=args)
#     node = SquareWallFollower()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SquareWallFollower()
        
        # Use MultiThreadedExecutor to handle callbacks concurrently
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
            
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()




