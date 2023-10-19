#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
# ! important for Parameter to be of from rcl_interfaces.msg 
from rcl_interfaces.msg import ParameterValue, ParameterType, Parameter
from rcl_interfaces.srv import SetParameters

class BorderColorChangeNode(Node):
    def __init__(self):
        super().__init__('border_color_node')
        # subscription used to get location from turtle running in simulator
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.cli = self.create_client(SetParameters, '/turtlesim/set_parameters')
        self.req = SetParameters.Request()

    # request sending asynchronous function for background change
    def send_request(self, param_b, value_b, param_g, value_g, param_r, value_r):
        new_param_value_b = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(value_b))
        new_param_value_g = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(value_g))
        new_param_value_r = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(value_r))
        self.req.parameters = [Parameter(name=param_b, value=new_param_value_b),
                               Parameter(name=param_g, value=new_param_value_g),
                               Parameter(name=param_r, value=new_param_value_r)]
        self.future = self.cli.call_async(self.req)

    # check for turtle position, if it intersected with any walls
    def pose_callback(self, msg: Pose):
        if msg.x <= 0.1:  # Left border
            self.change_background_color("left", 255, 0, 0)  # Blue
        elif msg.x >= 10.9:  # Right border
            self.change_background_color("right", 0, 255, 0)  # Green
        elif msg.y <= 0.1:  # Bottom border
            self.change_background_color("bottom", 0, 165, 255)  # Orange
        elif msg.y >= 10.9:  # Top border
            self.change_background_color("top", 0, 0, 255)  # Red

    # used to send the background color requests and logging info
    def change_background_color(self, side, b: int, g: int, r: int):
        self.send_request('background_b', b, 'background_g', g, 'background_r', r)

        self.get_logger().info(f'Background color changed to {side} ({r}, {g}, {b})')

def main(args=None):
    rclpy.init(args=args)
    node = BorderColorChangeNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()