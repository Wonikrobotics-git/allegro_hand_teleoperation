#!/usr/bin/env python3
# Copyright (c) 2025 WonikRobotics_official
# Additional Contributions Copyright (c) 2025 Jungyeon Lee
#
# This software is licensed under the MIT License.
# See the LICENSE file in the project root for full license text.

"""
Simple ROS2 node that forwards external 16-D Float64MultiArray commands
directly to the Allegro hand position controller.

- Parameters:
  - controller_name (string): name of the controller (default: allegro_hand_position_controller)
  - command_topic   (string): topic to subscribe for incoming commands (default: /finger_command)
- Incoming message type: std_msgs/Float64MultiArray (expects 16 values)
- Outgoing: published to /{controller_name}/commands (Float64MultiArray)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import ListControllers, SwitchController
import numpy as np

# 25.11.05 Avery
# This node is to forward the 16-D command to allegro hand controller
# This is for using with teleoperation of both hands
# This is for new allegro package which uses Float64MultiArray for command


def activate_controller(node: Node, controller_to_activate: str) -> bool:
    """
    Activates a specified controller via the controller manager if it's not already active.

    This function first checks the current state of all available controllers.
    If the target controller is already 'active', it returns True immediately.
    Otherwise, it sends a request to the '/controller_manager/switch_controller'
    service to activate the desired controller.

    Args:
        node: The ROS 2 node instance from which to create service clients.
        controller_to_activate: The name of the controller to be activated.

    Returns:
        True if the controller is or becomes active, False otherwise.
    """
    # First, check the current state of the controller to avoid unnecessary service calls.
    list_controllers_client = node.create_client(
        ListControllers, '/controller_manager/list_controllers'
    )
    if not list_controllers_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(
            "Could not connect to /controller_manager/list_controllers service."
        )
        return False

    list_req = ListControllers.Request()
    future = list_controllers_client.call_async(list_req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    if future.result():
        for controller in future.result().controller:
            if controller.name == controller_to_activate:
                if controller.state == "active":
                    node.get_logger().info(
                        f"Controller '{controller_to_activate}' is already active."
                    )
                    return True
                # Controller found, but not active. Break and proceed to activate.
                break
    else:
        # This is not a fatal error; we can still attempt to activate the controller.
        node.get_logger().warn("Failed to get controller list. Proceeding with activation attempt.")

    # If the controller is not already active, request to switch to it.
    switch_controller_client = node.create_client(
        SwitchController, '/controller_manager/switch_controller'
    )

    if not switch_controller_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(
            "Could not connect to /controller_manager/switch_controller service."
        )
        return False

    req = SwitchController.Request()
    req.activate_controllers = [controller_to_activate]
    # STRICT ensures that the activation will fail if any other controller
    # claims the same resources.
    req.strictness = SwitchController.Request.STRICT

    node.get_logger().info(f"Activating controller: '{controller_to_activate}'...")

    future = switch_controller_client.call_async(req)

    # Block until the service call is complete or times out.
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    if future.result() is not None:
        if future.result().ok:
            node.get_logger().info(f"Successfully activated controller '{controller_to_activate}'")
            return True
        else:
            node.get_logger().error(f"Failed to activate controller '{controller_to_activate}'")
            return False
    else:
        node.get_logger().error(
            f"Service call to activate controller '{controller_to_activate}' timed out."
        )
        return False


class AllegroCommandForwarder(Node):
    def __init__(self, side:str='right', one_hand:bool=False, two_robots:bool=True):
        """
        Initialize AllegroCommandForwarder.

        Args:
            side: Which hand ('left' or 'right')
            one_hand: If True, control only one hand (ignored if two_robots=False)
            two_robots: If True, system has two robots (use _l/_r suffix).
                       If False, system has single robot (no suffix).
        """
        super().__init__('allegro_command_forwarder')

        if not two_robots:
            # Single robot system: no suffix in controller name
            self.get_logger().info("Single robot mode: using controller without suffix")
            self.declare_parameter('controller_name', 'allegro_hand_position_controller')
            self.declare_parameter('command_topic', '/finger_command')

        else:
            # Two robots system: use _l/_r suffix
            if side == 'left':
                self.declare_parameter('controller_name', 'allegro_hand_position_controller_l')
                self.declare_parameter('command_topic', '/finger_command_l')
            else:
                self.declare_parameter('controller_name', 'allegro_hand_position_controller_r')
                self.declare_parameter('command_topic', '/finger_command_r')

        self.controller_name = self.get_parameter('controller_name').get_parameter_value().string_value
        self.command_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        self.target_topic = f'/{self.controller_name}/commands'

        # Try to activate controller
        if not activate_controller(self, self.controller_name):
            self.get_logger().error(f"Failed to activate controller '{self.controller_name}'. Node will not forward commands.")
            # Still continue to create subscription so user can retry or inspect logs,
            # but publisher will remain None and publishes will be skipped.
            self.publisher_ = None
        else:
            # Publisher to controller command topic
            self.publisher_ = self.create_publisher(Float64MultiArray, self.target_topic, 10)

        # Safe base position (neutral) to return to on shutdown
        self.base_position = np.array([
            0.5, 0.2, 0.0, 0.0,   # Thumb
            0.0, 0.0, 0.0, 0.0,   # Index
            0.0, 0.0, 0.0, 0.0,   # Middle
            0.0, 0.0, 0.0, 0.0,   # Ring
        ])

        # Subscriber: receive external commands and forward immediately
        self.create_subscription(Float64MultiArray, self.command_topic, self.command_cb, 10)

        self.get_logger().info(f'AllegroCommandForwarder started.')
        self.get_logger().info(f'Listening to command topic: {self.command_topic} (Float64MultiArray)')
        self.get_logger().info(f'Forwarding to controller topic: {self.target_topic}')

    def command_cb(self, msg: Float64MultiArray):
        """
        Receives external 16-D commands and forwards them to the controller.
        If incoming message doesn't have exactly 16 elements, ignore and warn.
        """
        # Validate message length
        data = list(msg.data) if msg and msg.data is not None else []
        if len(data) != 16:
            self.get_logger().warning(f"Ignoring incoming command: expected 16 elements but got {len(data)}.")
            return

        if not self.publisher_:
            self.get_logger().error("No publisher available (controller not activated). Cannot forward command.")
            return

        # Forward message as-is
        out = Float64MultiArray()
        out.data = data
        self.publisher_.publish(out)
        self.get_logger().debug("Forwarded 16-D command to controller.")

    def return_to_base(self):
        """Publish base (safe) position once (used at shutdown)."""
        if self.publisher_:
            msg = Float64MultiArray()
            msg.data = self.base_position.tolist()
            self.publisher_.publish(msg)
            self.get_logger().info("Published base (safe) position.")


def main(args=None):
    rclpy.init(args=args)
    node = AllegroCommandForwarder(side='right')  # Change to 'left' for left hand
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down AllegroCommandForwarder...")
        try:
            node.return_to_base()
        except Exception as e:
            node.get_logger().warn(f"Failed to publish base position on shutdown: {e}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

    # ros2 topic pub /finger_command_r std_msgs/Float64MultiArray "{data: [0.5,0.2,0.6,0.0, 0,0,0,0, 0,0,0,0, 0,0,0,0]}" -1


