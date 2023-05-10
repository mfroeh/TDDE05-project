import rclpy
import predict

from rclpy.node import Node
from std_msgs.msg import String

class NlpNode(Node):

    def __init__(self):
        super().__init__('nlp_node')
        self.subscription = self.create_subscription(
            String,
            '/input_prompt',
            self.receive_prompt,
            10)
        self.subscription

        self.get_logger().info('Initialization Completed')

    def receive_prompt(self, prompt):
        self.get_logger().info('I heard "%s"' % prompt.data)
        self.get_logger().info('I think "%s"' % self.recognize_intent(prompt))

    def recognize_intent(self, prompt):
        self.get_logger().info(predict("airproject_model", prompt))


def main(args=None):
    rclpy.init(args=args)

    nlp_node = NlpNode()
    rclpy.spin(nlp_node)
    nlp_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
