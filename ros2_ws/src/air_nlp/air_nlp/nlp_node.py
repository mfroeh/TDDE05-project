import rclpy
from .predict import predict, get_args, load_model, get_device
import os

from rclpy.node import Node
from std_msgs.msg import String

from air_interfaces.msg import Goal, Destination
from air_interfaces.action import Goals


def is_office(x):
    return x == "B-office"


def is_user(x):
    return x == "B-user"


def is_person(x):
    return x == "B-person"


def is_destination(x):
    return is_person(x) or is_user(x) or is_office(x)


class NlpNode(Node):

    def __init__(self):
        super().__init__('nlp_node')

        model_dir = os.path.expanduser(
            '~') + "/air-projects-03/ros2_ws/src/air_nlp/air_nlp/airproject_model/"
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

        words = []
        for word in prompt.data:
            words.append(word)

        self.get_logger().info('The intent is: "%s"' % intent)
        self.get_logger().info('The slots are:')

        for slot in slots:
            self.get_logger().info("{} ".format(slot))

        self.generate_goals(words, slots)

    def generate_goals(self, words, slots):
        goals = []
        known_destinations = []

        for i, (word, slot) in enumerate(zip(words, slots)):
            if slot == "B-goal.goto":
                goal = Goal()
                goal.type = "goto"
                try:
                    destination = Destination()
                    destination.type = "office" if is_office(slot) else "person"
                    known_destinations.append(destination.name)
                    goal.destination = destination
                    goals.append(goal)
                except StopIteration:
                    self.get_logger().error("Could not find a destination for goto goal")

        for goal in goals:
            print("Goal of type: %s to destination: %s (%s)" % (goal.type, goal.destination.name, goal.destination.type))


def main(args=None):
    rclpy.init(args=args)

    nlp_node = NlpNode()
    rclpy.spin(nlp_node)
    nlp_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
