import rclpy
from rclpy.node import Node
from tensorflow import keras

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
        self.model = keras.models.load_model('model')
        self.get_logger().info('Initialization Completed')

    def receive_prompt(self, prompt):
        self.get_logger().info('I heard "%s"' % prompt.data)


def main(args=None):
    rclpy.init(args=args)

    nlp_node = NlpNode()
    rclpy.spin(nlp_node)
    nlp_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
