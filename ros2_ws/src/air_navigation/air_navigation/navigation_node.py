import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

#from nav2_simple_commander.costmap_2d import PyCostmap2D

import tf2_geometry_msgs

import json

from array import array
import numpy as np

import ament_index_python

from std_msgs.msg import String,Header
from air_interfaces.msg import People,Person
from air_simple_sim_msgs.msg import SemanticObservation


class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.msgPeople = People()
        self.header=Header()
        self.header.frame_id = "map"
        self.msgPeople.header = self.header
        self.people_publisher = self.create_publisher(People,
            '/people', 9)

        self.people_subscription = self.create_subscription(
            SemanticObservation,
            '/semantic_sensor_hf',
            self.listener_callback,
            10)

        self.previous_costmap_data = None


    def listener_callback(self, msg):#2500 local 50*50
            #if msg.uuid != self.oldMsg:
        msgPerson=Person()
        if msg.klass=='human':

            msgPerson.name = msg.tags[0]
            msg.point.header.stamp = rclpy.time.Time().to_msg()
            try:
                point_transformed = self.tf_buffer.transform(msg.point, "map",timeout=rclpy.duration.Duration(seconds=1.0))
            except TransformException as ex:
                self.get_logger().info(f'Could not transform {msg.point} to {"map"}: {ex}')
                return
            #msg.point = point_transformed
            
            msgPerson.position = point_transformed.point
            msgPerson.velocity.x=0.0
            msgPerson.velocity.y=0.0
            msgPerson.velocity.z=0.0
            if self.isUpdata(msgPerson):
                pass
            else:
                self.msgPeople.people.append(msgPerson)

        self.msgPeople.header.stamp = msg.point.header.stamp #self.get_clock().now().to_msg()
        self.people_publisher.publish(self.msgPeople)
        print(self.msgPeople)
        return 
    
        # Checking print:
        # print("init y: ",msg.pose.pose.position.y)

    def isUpdata(self,msgPerson):
        for person in self.msgPeople.people:
            if msgPerson.name == person.name:
                person = msgPerson#update people info
                return True
        return False



def main(args=None):
    rclpy.init(args=args)

    navigation_node = NavigationNode()

    rclpy.spin(navigation_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()