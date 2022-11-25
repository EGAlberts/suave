# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Optional

import rclpy

# Node, State and Publisher are aliases for LifecycleNode, LifecycleState and LifecyclePublisher
# respectively.
# In case of ambiguity, the more explicit names can be imported.

from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer

import std_msgs.msg
import numpy as np

def spiral_points(i, resolution=0.1, spiral_width=1.0):
    step_idx = resolution * i
    x = np.cos(step_idx) * spiral_width * step_idx 
    y = np.sin(step_idx) * spiral_width * step_idx 
    return x,y

class LifecycleTalker(Node):
    """Our lifecycle talker node."""

    def __init__(self, node_name, **kwargs):
        """Construct the node."""
        self._count: int = 0
        self._pub: Optional[Publisher] = None
        self._timer: Optional[Timer] = None
        super().__init__(node_name, **kwargs)
        self.spiral_width: float = 0.
        self.get_logger().info('node initiated '+node_name)

        param_descriptor = ParameterDescriptor(
            description='Sets the spiral width of the UUV.')
        self.declare_parameter('spiral_width', 1.0, param_descriptor)        

        self.param_change_callback_handle_ = self.add_on_set_parameters_callback(
                self.param_change_callback)

    def param_change_callback(self, parameters):
        result = SetParametersResult()
        result.successful = True
        for parameter in parameters:
            self.get_logger().info(
                "parameter '{}' is now: {}".format(
                    parameter.name,
                    parameter.value))
        return result


    def pose_stamped(
            self, x=.0, y=.0, z=.0, rx=.0, ry=.0, rz=.0, rw=1.0):

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = rx
        pose.pose.orientation.y = ry
        pose.pose.orientation.z = rz
        pose.pose.orientation.w = rw

        return pose


    def publish(self):
        """Publish a new message when enabled."""
        print('entered publisher')
        x,y = spiral_points(self._count, resolution=0.1,  spiral_width=self.spiral_width)
        msg = self.pose_stamped(x,y)
        self._count += 1

        self.get_logger().info(
                'setpoint_postion_local value {}'.format(
                    msg.pose.position))
        
        # Print the current state for demo purposes
        if self._pub is None or not self._pub.is_activated:
            self.get_logger().info(
                    'Lifecycle publisher is currently inactive. Messages are not published.')
        else:
            self.get_logger().info(
                    f'Lifecycle publisher is active. Publishing: [{msg.pose.position}]')

        # We independently from the current state call publish on the lifecycle
        # publisher.
        # Only if the publisher is in an active state, the message transfer is
        # enabled and the message actually published.
        if self._pub is not None:
            self._pub.publish(msg)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Configure the node, after a configuring transition is requested.

        on_configure callback is being called when the lifecycle node
        enters the "configuring" state.

        :return: The state machine either invokes a transition to the "inactive" state or stays
        in "unconfigured" depending on the return value.
          TransitionCallbackReturn.SUCCESS transitions to "inactive".
          TransitionCallbackReturn.FAILURE transitions to "unconfigured".
          TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        self.get_logger().info('entered on_configure')
        self.spiral_width = self.get_parameter(
                'spiral_width').get_parameter_value().double_value
        self.get_logger().info(
                'spiral width is {}'.format(
                    self.spiral_width))
        self._pub = self.create_lifecycle_publisher(PoseStamped,
                "mavros/setpoint_position/local", 10);
        self.get_logger().info('publisher created')
        self._timer_ = self.create_timer(1.0, self.publish)
        self.get_logger().info('timer created')

        self.get_logger().info("on_configure() is called.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Differently to rclcpp, a lifecycle publisher transitions automatically between the inactive and
        # enabled state and viceversa.
        # For that reason, we only need to write an on_configure() and on_cleanup() callbacks, and we don't
        # need to write on_activate()/on_deactivate() callbacks.

        # Log, only for demo purposes
        self.get_logger().info("on_activate() is called.")

        # The default LifecycleNode callback is the one transitioning
        # LifecyclePublisher entities from inactive to enabled.
        # If you override on_activate(), don't forget to call the parent class method as well!!
        return super().on_activate(state)
  
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # Log, only for demo purposes
        self.get_logger().info("on_deactivate() is called.")
        # Same reasong here that for on_activate().
        # These are the two only cases where you need to call the parent method.
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """
        Cleanup the node, after a cleaning-up transition is requested.

        on_cleanup callback is being called when the lifecycle node
        enters the "cleaning up" state.

        :return: The state machine either invokes a transition to the "unconfigured" state or stays
        in "inactive" depending on the return value.
          TransitionCallbackReturn.SUCCESS transitions to "unconfigured".
          TransitionCallbackReturn.FAILURE transitions to "inactive".
          TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        self.destroy_timer(self._timer)
        self.destroy_publisher(self._pub)

        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """
        Shutdown the node, after a shutting-down transition is requested.

        on_shutdown callback is being called when the lifecycle node
        enters the "shutting down" state.

        :return: The state machine either invokes a transition to the "finalized" state or stays
        in the current state depending on the return value.
          TransitionCallbackReturn.SUCCESS transitions to "unconfigured".
          TransitionCallbackReturn.FAILURE transitions to "inactive".
          TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        self.destroy_timer(self._timer)
        self.destroy_publisher(self._pub)

        self.get_logger().info('on_shutdown() is called.')
        return TransitionCallbackReturn.SUCCESS


# A lifecycle node has the same node API
# as a regular node. This means we can spawn a
# node, give it a name and add it to the executor.

def main():
    rclpy.init()

    executor = rclpy.executors.SingleThreadedExecutor()
    lc_node = LifecycleTalker('spiral_lc_node')
    print('node instantiated')
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()

if __name__ == '__main__':
    main()
