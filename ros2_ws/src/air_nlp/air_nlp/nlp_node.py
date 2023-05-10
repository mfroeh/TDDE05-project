import rclpy
from .predict import predict, get_args, load_model, get_device

from rclpy.node import Node
from std_msgs.msg import String

from air_interfaces import Destination, Goal, Goals

class NlpNode(Node):

    def __init__(self):
        super().__init__('nlp_node')

        model_dir = "./airproject_model/"
        self.args = get_args(model_dir)
        self.device = get_device()
        self.model = load_model(model_dir, self.args, self.device)

        self.subscription = self.create_subscription(
            String,
            '/input_prompt',
            self.receive_prompt,
            10)

        self.get_logger().info('Initialization Completed')

    def receive_prompt(self, prompt):
        self.get_logger().info('Received prompt: "%s"' % prompt.data)

        (intent, slots) = predict(prompt.data, self.model, self.args, self.device)

        self.get_logger().info('The intent is: "%s"' % intent)

        for slot in slots:
            self.get_logger().info("{} ".format(slot))

    def generate_goals(self, slots):
        if 'B-goal.goto' in slots:
            self.get_logger().info("Goto")




def main(args=None):
    rclpy.init(args=args)

    nlp_node = NlpNode()
    rclpy.spin(nlp_node)
    nlp_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
