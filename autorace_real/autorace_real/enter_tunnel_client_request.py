# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# from example_interfaces.srv import Trigger
# from nav2_msgs.action import NavigateToPose
# from geometry_msgs.msg import PoseStamped

# from rclpy.action import ActionClient


# class EnterTunnelReal(Node):
#     def __init__(self):
#         super().__init__('enter_tunnel_real')
#         self.get_logger().info('enter_tunnel_real node started.')

#         # QoS for service client: reliable, keep last 10 msgs
#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.RELIABLE,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=10)

#         # Service client to detect tunnel sign
#         self.sign_client = self.create_client(Trigger, 'detect_tunnel_sign', qos_profile=qos_profile)
#         self.get_logger().info('Waiting for detect_tunnel_sign service...')
#         if not self.sign_client.wait_for_service(timeout_sec=5.0):
#             self.get_logger().error('detect_tunnel_sign service not available. Exiting...')
#             rclpy.shutdown()
#             return
#         self.get_logger().info('Service is ready.')

#         # Nav2 action client
#         self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
#         self.get_logger().info('Waiting for navigate_to_pose action server...')
#         if not self.nav_client.wait_for_server(timeout_sec=5.0):
#             self.get_logger().error('navigate_to_pose action server not available. Exiting...')
#             rclpy.shutdown()
#             return
#         self.get_logger().info('Nav2 action server ready.')

#         # Use a timer instead of blocking while-loop
#         self.timer = self.create_timer(2.0, self.check_sign_and_navigate)
#         self.sign_detected = False

#     def check_sign_and_navigate(self):
#         if self.sign_detected:
#             # Already detected and navigating, stop timer
#             self.timer.cancel()
#             return

#         self.get_logger().info('Checking for tunnel sign detection...')
#         if self.call_sign_service():
#             self.get_logger().info('Tunnel sign detected! Proceeding to navigation.')
#             self.sign_detected = True
#             self.send_nav_goal()
#         else:
#             self.get_logger().info('Sign not detected yet. Will retry.')

#     def call_sign_service(self):
#         request = Trigger.Request()
#         future = self.sign_client.call_async(request)
#         rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

#         if future.done() and future.result() is not None:
#             response = future.result()
#             if response.success:
#                 self.get_logger().info(f'Service response: {response.message}')
#                 return True
#             else:
#                 self.get_logger().info(f'Service says no sign yet: {response.message}')
#         else:
#             self.get_logger().error('Service call failed or timed out.')
#         return False

#     def send_nav_goal(self):
#         goal_msg = NavigateToPose.Goal()

#         pose = PoseStamped()
#         pose.header.frame_id = 'map'
#         pose.header.stamp = self.get_clock().now().to_msg()

#         # Destination coordinates - change as needed
#         pose.pose.position.x = 2.0
#         pose.pose.position.y = 0.0
#         pose.pose.orientation.w = 1.0

#         goal_msg.pose = pose

#         self.get_logger().info('Sending Nav2 goal...')
#         send_goal_future = self.nav_client.send_goal_async(goal_msg)

#         # Wait for goal acceptance asynchronously
#         rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)

#         goal_handle = send_goal_future.result()
#         if not goal_handle or not goal_handle.accepted:
#             self.get_logger().error('Goal was rejected by Nav2 or timed out.')
#             return

#         self.get_logger().info('Goal accepted! Waiting for result...')
#         get_result_future = goal_handle.get_result_async()
#         rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=60.0)

#         result = get_result_future.result()
#         if result:
#             self.get_logger().info(f'Navigation finished with result: {result.result}')
#         else:
#             self.get_logger().error('Failed to get navigation result or timed out.')


# def main(args=None):
#     rclpy.init(args=args)
#     node = EnterTunnelReal()

#     # Keep spinning until node shuts down
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass

#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from example_interfaces.srv import Trigger

class EnterTunnelSimpleClient(Node):
    def __init__(self):
        super().__init__('enter_tunnel_simple_client')
        self.get_logger().info('enter_tunnel_simple_client node started.')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.sign_client = self.create_client(Trigger, 'detect_tunnel_sign', qos_profile=qos_profile)
        self.get_logger().info('Waiting for detect_tunnel_sign service...')

        if not self.sign_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('detect_tunnel_sign service not available. Exiting...')
            rclpy.shutdown()
            return

        self.timer = self.create_timer(2.0, self.check_sign)

    def check_sign(self):
        self.get_logger().info('Checking for tunnel sign...')
        request = Trigger.Request()
        future = self.sign_client.call_async(request)

        def callback(fut):
            if fut.result() and fut.result().success:
                self.get_logger().info(f"Tunnel sign detected! Service says: {fut.result().message}")
                self.timer.cancel()  # stop further checking
            else:
                self.get_logger().info('No sign detected yet. Will retry.')

        future.add_done_callback(callback)

def main(args=None):
    rclpy.init(args=args)
    node = EnterTunnelSimpleClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
