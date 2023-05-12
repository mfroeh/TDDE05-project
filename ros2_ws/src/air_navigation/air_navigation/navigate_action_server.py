import rclpy
from rclpy.action import ActionServer,ActionClient
from rclpy.node import Node
from rclpy.node import Node
from tf2_ros import TransformException, Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf_transformations
from air_interfaces.action import Navigate
from nav2_msgs.action import NavigateToPose



class NavigateActionServer(Node):

    def __init__(self):
        super().__init__('navigate_action_server')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._action_server = ActionServer(
            self,
            Navigate,
            'navigate',
            self.execute_callback)
        

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        #handle request

        target = goal_handle.request.position

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = target.x
        goal_msg.pose.pose.position.y = target.y

        #transmit the target to Nav2 
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        

        #handle feedback
        feedback_msg = Navigate.Feedback()


        #handle result
        goal_handle.succeed()
        result = Navigate.Result()
        
        return result

    def get_robot_pose(self):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()

        try:
            # Get the robot's pose in the 'map' frame
            transformed_pose = self.tf_buffer.transform(pose_stamped, "map", timeout=rclpy.duration.Duration(seconds=1.0))
        except TransformException as ex:
            self.get_logger().error(f'Failed to get robot pose: {ex}')
            return None

        robot_x = transformed_pose.pose.position.x
        robot_y = transformed_pose.pose.position.y
        _, _, robot_yaw = tf_transformations.euler_from_quaternion([
            transformed_pose.pose.orientation.x,
            transformed_pose.pose.orientation.y,
            transformed_pose.pose.orientation.z,
            transformed_pose.pose.orientation.w
        ])

        return robot_x, robot_y, robot_yaw


def main(args=None):
    rclpy.init(args=args)

    navigate_action_server = NavigateActionServer()

    rclpy.spin(navigate_action_server)


if __name__ == '__main__':
    main()