import rclpy
from .predict import predict, get_args, load_model, get_device
import os
from word2number import w2n

from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String

from air_interfaces.msg import Goal, Destination
from air_interfaces.action import Goals


def is_office(x):
    return x == "B-office"


def is_user(x):
    return x == "B-user"


def is_person(x):
    return x == "B-person"


def is_everyone(x):
    return x == "B-everyone"


def is_object(x):
    return x == "B-object"


def is_quantity(x):
    return x == "B-quantity"

def is_goal(x):
    return "goal" in x

def is_destination(x):
    return is_person(x) or is_user(x) or is_office(x)


def get_destination_type(x):
    if is_destination(x):
        return remove_prefix(x, 'B-')
    else:
        return "invalid"


def create_destination(word, slot):
    destination = Destination()
    destination.name = remove_suffix(word, "'s").capitalize()
    destination.type = get_destination_type(slot)
    return destination


def find_destination(i, words, slots, stop=False):
    for j, (word, slot) in enumerate(zip(words[i:], slots[i:])):
        if is_goal(slot) and stop:
            return None
        if is_destination(slot):
            destination = create_destination(word, slot)
            slots.pop(j + i)
            words.pop(j + i)
            return destination
    return None


def create_object(word):
    object = remove_suffix(word, "s")
    if object == "sandwiche":
        return "sandwich"
    else:
        return object


def find_object(i, words, slots):
    for j, (word, slot) in enumerate(zip(words[i:], slots[i:])):
        if is_object(slot):
            object = create_object(word)
            slots.pop(j + i)
            words.pop(j + i)
            return object
    return None


def create_quantity(word):
    if word == "a" or word == "an" or word == "some" or word == "one" or word == "1":
        return 1
    else:
        try:
            return int(word)
        except Exception:
            try:
                return w2n.word_to_num(word)
            except Exception:
                return 1


def find_quantity(i, words, slots):
    for j, (word, slot) in enumerate(zip(words[i:], slots[i:])):
        if is_quantity(slot):
            return_value = create_quantity(word)
            words.pop(j + i)
            slots.pop(j + i)
            return return_value
    return 1


def is_there_everyone(slots):
    for slot in slots:
        if is_everyone(slot):
            return True
    return False


def remove_suffix(s, suffix):
    if suffix and s.endswith(suffix):
        return s[:-len(suffix)]
    return s


def remove_prefix(s, prefix):
    if prefix and s.startswith(prefix):
        return s[len(prefix):]
    return s


def add_known_person(name, people):
    person = Destination()
    person.type = "person"
    person.name = name
    people.append(person)


def add_bring_goal(object, destination, goals):
    goal = Goal()
    goal.type = "bring"
    goal.object = object
    goal.destination = destination
    goals.append(goal)


def add_bring_goals(object, destination, quantity, goals):
    for k in range(quantity):
        add_bring_goal(object, destination, goals)


def add_bring_everyone(object, destination, goals, known_people):
    destination = Destination()
    destination.type = "user"
    destination.name = "N/A"
    add_bring_goal(object, destination, goals)
    for destination in known_people:
        add_bring_goal(object, destination, goals)


def add_goto_goal(destination, goals):
    goal = Goal()
    goal.type = "goto"
    goal.destination = destination
    goals.append(goal)


class NlpNode(Node):

    def __init__(self):
        super().__init__('nlp_node')
        self.get_logger().info('Starting Initialization')

        model_dir = os.path.expanduser(
            '~') + "/TDDE05/ros2_ws/src/air_nlp/air_nlp/airproject_model/"
        self.args = get_args(model_dir)
        self.device = get_device()
        self.model = load_model(model_dir, self.args, self.device)

        self._action_client = ActionClient(self, Goals, '/goals_request')

        self.subscription = self.create_subscription(
            String,
            '/input_prompt',
            self.receive_prompt,
            10)

        self.get_logger().info('Initialization Completed')

    def receive_prompt(self, prompt):
        self.get_logger().info('Received prompt: "%s"' % prompt.data)

        (intent, slots) = predict(prompt.data, self.model, self.args, self.device)

        line = prompt.data.lower()

        words = []
        for word in line.split():
            words.append(word)

        self.get_logger().info('The intent is: "%s"' % intent)
        self.get_logger().info('The slots are:')

        for slot in slots:
            self.get_logger().info("{} ".format(slot))

        goals = self.generate_goals(words, slots)
        if len(goals):
            self.send_goals(goals)
        else:
            self.get_logger().info("Could not find goals from prompt")

    def send_goals(self, goals):
        goal_msg = Goals.Goal()
        goal_msg.goals = goals
        self.get_logger().info("Waiting for server to send goals")
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {}'.format(result.success))

    def generate_goals(self, words, slots):
        goals = []  # Current created goals
        known_people = []  # People encountered so far

        i = 0
        while i < len(slots):
            # Loop over the slots and the words

            slot = slots[i]
            word = words[i]

            # Handle a goto goal

            if slot == "B-goal.goto":
                # Finds and pops the nearest destination
                destination = find_destination(i, words, slots)
                if destination is None:
                    raise Exception("Could not find a destination for goto")
                else:
                    add_goto_goal(destination, goals)
                    add_known_person(destination.name, known_people)

            # Handle a bring goal

            elif slot == "B-goal.bring":

                object = find_object(i, words, slots)  # Finds and pops the nearest object
                destination = find_destination(i, words, slots)

                if destination:
                    # If a destination is found, find the quantity and generate bring goals

                    quantity = find_quantity(i, words, slots)
                    add_known_person(destination.name, known_people)

                    add_bring_goals(object, destination, quantity, goals)
                else:
                    if is_there_everyone(slots[i:]):
                        # If a B-everyone tag is found, create a bring goal for user and every known person
                        add_bring_everyone(object, destination, goals, known_people)
                    else:
                        if known_people:
                            # If there are known people and no destination is found
                            # generate the bring goal for the latest known person

                            quantity = find_quantity(i, words, slots)

                            add_bring_goals(object, known_people[-1], quantity, goals)
                        else:
                            raise Exception("Could not find a destination for bring")

            # Handle left over destinations by cloning latest goal

            elif is_destination(slot) and len(goals):
                last_goal = goals[-1]
                destination = create_destination(word, slot)
                add_known_person(destination.name, known_people)
                if last_goal.type == "goto":
                    add_goto_goal(destination, goals)
                else:  # last_goal is "bring"
                    object = find_object(i, words, slots)
                    if object:
                        quantity = find_quantity(i, words, slots)
                        add_bring_goals(object, destination, quantity, goals)
                    else:
                        add_bring_goal(last_goal.object, destination, goals)

            # Handle left over objects by cloning latest bring goal

            elif is_object(slot) and len(goals):
                last_goal = goals[-1]
                if last_goal.type != "bring":
                    raise Exception("I could not understand composite goals")
                destination = find_destination(i, words, slots)
                if destination is None:
                    destination = last_goal.destination
                add_bring_goal(create_object(word), destination, goals)

            elif is_quantity(slot) and len(goals):
                last_goal = goals[-1]
                if last_goal.type != "bring":
                    raise Exception("I could not understand composite goals")
                quantity = create_quantity(word)
                object = find_object(i, words, slots)
                if object:
                    add_bring_goals(object, last_goal.destination, quantity, goals)
                else:
                    # True means stop searching if you encounter a new goal
                    destination = find_destination(i, words, slots, True)
                    add_bring_goals(last_goal.object, destination, quantity, goals)
            i += 1

        for goal in goals:
            if goal.type == "goto":
                self.get_logger().info("Goal: go to %s (%s)" % (goal.destination.name,
                                                                goal.destination.type))
            else:
                self.get_logger().info("Goal: bring %s to %s (%s)" %
                                       (goal.object, goal.destination.name, goal.destination.type))

        return goals


def main(args=None):
    rclpy.init(args=args)

    nlp_node = NlpNode()
    rclpy.spin(nlp_node)
    nlp_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
