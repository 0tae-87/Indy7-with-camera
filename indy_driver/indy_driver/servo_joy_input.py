#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
# from std_srvs.srv import Trigger
from moveit_msgs.srv import ServoCommandType 

import signal
import time

from indy_interfaces.srv import IndyService
from indy_define import *

# For XBOX S controller
class Axis:
    LEFT_STICK_X = 0
    LEFT_STICK_Y = 1
    LEFT_TRIGGER = 5
    RIGHT_STICK_X = 2
    RIGHT_STICK_Y = 3
    RIGHT_TRIGGER = 4
    D_PAD_X = 6
    D_PAD_Y = 7

class Button:
    A = 0
    B = 1
    X = 4
    Y = 3
    LEFT_BUMPER = 6
    RIGHT_BUMPER = 7
    CHANGE_VIEW = 10
    MENU = 11
    XBOX = 12
    HOME = 15
    LEFT_STICK_CLICK = 13
    RIGHT_STICK_CLICK = 14

# Some axes have offsets (e.g. the default trigger position is 1.0 not 0)
# This will map the default values for the axes
AXIS_DEFAULTS = { Axis.LEFT_TRIGGER: 1.0, Axis.RIGHT_TRIGGER: 1.0 }
BUTTON_DEFAULTS = {}

EEF_FRAME_ID        = "tcp"
BASE_FRAME_ID       = "link0"
ROS_QUEUE_SIZE      = 10

# -----TELE STATUS-----
TELE_STOP   = 0
TELE_TASK   = 1
TELE_JOINT  = 2

# -----MAX SPEED MULTIPLY (FOR SAFETY)----
MAX_SPEED_MUL = 0.5

class JoyToServoPub(Node):

    def __init__(self):
        super().__init__('servo_joy_input')

        # Create a service client to start the ServoNode
        self.servo_client = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
        while not self.servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('servo service not available, waiting again...')
        self.servo_cmd_type = ServoCommandType.Request()

        # parameters
        self.declare_parameter('is_sim', True)
        # self.declare_parameter('robot_name', 'indy7')
        # self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.isSim = self.get_parameter('is_sim').get_parameter_value().bool_value

        # service client for indy
        if not self.isSim:
            self.cli = self.create_client(IndyService, 'indy_srv')
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('indy_srv service not available, waiting again...')
            self.indy_req = IndyService.Request()

        self.teleop_status      = TELE_STOP
        self.frame_to_publish   = BASE_FRAME_ID

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, ROS_QUEUE_SIZE)
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', ROS_QUEUE_SIZE)
        self.joint_pub = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', ROS_QUEUE_SIZE)
        
        self.future_callback_waiting = None
        self.servo_service()

        print("TESTED WITH XBOX ONE S CONTROLLER")
        print("DUE TO LACK OF BUTTON ON XBOX CONTROLLER, THE DEFAULT SERVO MODE WILL BE TWIST")
        print("PLEASE TURN ON TELE MODE BY PRESS 'HOME BUTTON' IF YOU ARE USING WITH REAL ROBOT")
        print("---------For Joint Control---------")
        print("Using Dpad to control joint 1 and joint 2.")
        print("B and X control joint 4")
        print("Y and A control joint 3")
        print("---------For Task Control---------")
        print("Left joystick, right joystick, LB, RB, LT, RT to control TCP.")
        print("---------Only Real Robot----------")
        print("Use 'LEFT_STICK_CLICK' to move Home, 'RIGHT_STICK_CLICK' to move Zero, 'XBOX' to Recover, 'HOME' to Start/Stop Teleop")

    def close(self):
        print("\nON EXIT.......")
        if not self.isSim:
            self.indy_service(MSG_TELE_STOP)
            time.sleep(0.1)

    def servo_service(self):
        self.servo_cmd_type.command_type = ServoCommandType.Request.TWIST
        self.get_logger().info("Switching to input type: Twist")
        self.servo_service_future = self.servo_client.call_async(self.servo_cmd_type)
        self.servo_service_future.add_done_callback(self.servo_service_future_callback)
        
    def servo_service_future_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("Switching Done!")
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))
            self.get_logger().info("Could not switch input")

    def indy_service(self, data):
        self.indy_req.data = data
        self.future = self.cli.call_async(self.indy_req)
        self.future.add_done_callback(self.future_callback)
        self.future_callback_waiting = data

    def future_callback(self, future):
        try:
            response = future.result()
            callback_map = {
                MSG_MOVE_HOME:      ("Call Move Home",  TELE_STOP),
                MSG_MOVE_ZERO:      ("Call Move Zero",  TELE_STOP),
                MSG_RECOVER:        ("Call Recover",    TELE_STOP),
                MSG_TELE_STOP:      ("Call Tele Stop",  TELE_STOP),
                MSG_TELE_JOINT_ABS: ("Call Tele Start", TELE_JOINT),
            }
            if self.future_callback_waiting in callback_map:
                action_msg, success_status = callback_map[self.future_callback_waiting]
                if response:
                    self.get_logger().info(f"{action_msg} Success!")
                    self.teleop_status = success_status
                else:
                    self.get_logger().info(f"{action_msg} Failed!")
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

        self.future_callback_waiting = None

    def isValid(self, mode):
        if self.isSim:
            return True
        elif not self.isSim:
            if (mode == TELE_TASK or mode == TELE_JOINT) and self.teleop_status != TELE_JOINT:
                # print("Please check if Tele Mode is On")
                return False
            else:
                return True
        else:
            return False

    def joy_callback(self, msg):
        # Create the messages and variables
        twist_msg = TwistStamped()
        joint_msg = JointJog()
        publish_twist = False
        publish_joint = False

        # This call updates the frame for twist commands
        if msg.buttons[Button.CHANGE_VIEW] and self.frame_to_publish == EEF_FRAME_ID:
            self.frame_to_publish = BASE_FRAME_ID
            print("CHANGE_VIEW link0")
        elif msg.buttons[Button.MENU] and self.frame_to_publish == BASE_FRAME_ID:
            self.frame_to_publish = EEF_FRAME_ID
            print("CHANGE_VIEW tcp")

        # this command only work with real robot
        if not self.isSim:
            if msg.buttons[Button.LEFT_STICK_CLICK]:
                if self.future_callback_waiting is None:
                    self.indy_service(MSG_MOVE_HOME)

            elif msg.buttons[Button.RIGHT_STICK_CLICK]:
                if self.future_callback_waiting is None:
                    self.indy_service(MSG_MOVE_ZERO)

            elif msg.buttons[Button.XBOX]:
                if self.future_callback_waiting is None:
                    self.indy_service(MSG_RECOVER)

            elif msg.buttons[Button.HOME]:
                if self.future_callback_waiting is None:
                    if self.teleop_status == TELE_STOP:
                        self.indy_service(MSG_TELE_JOINT_ABS)
                    elif self.teleop_status == TELE_JOINT:
                        self.indy_service(MSG_TELE_STOP)

        # if joint command
        if msg.buttons[Button.A] or msg.buttons[Button.B] or msg.buttons[Button.X] or \
            msg.buttons[Button.Y] or msg.axes[Axis.D_PAD_X] or msg.axes[Axis.D_PAD_Y]:
            if self.isValid(TELE_JOINT):
                # Map the D_PAD to the proximal joints
                joint_msg.joint_names = ['joint1', 'joint2']
                joint_msg.velocities = [msg.axes[Axis.D_PAD_X], msg.axes[Axis.D_PAD_Y]]

                # Map the diamond to the distal joints
                joint_msg.joint_names.extend(['joint4', 'joint3'])
                joint_msg.velocities.extend([msg.buttons[Button.B] - msg.buttons[Button.X], msg.buttons[Button.Y] - msg.buttons[Button.A]])

                publish_joint = True

        # task command
        elif msg.axes[Axis.RIGHT_STICK_X] or msg.axes[Axis.RIGHT_STICK_Y] or msg.axes[Axis.RIGHT_TRIGGER] or \
            msg.axes[Axis.LEFT_TRIGGER] or msg.buttons[Button.RIGHT_BUMPER] or msg.buttons[Button.LEFT_BUMPER]:
            # The bread and butter: map buttons to twist commands
            if self.isValid(TELE_TASK):
                lin_x_right = -0.5  * (msg.axes[Axis.RIGHT_TRIGGER] - AXIS_DEFAULTS[Axis.RIGHT_TRIGGER])
                lin_x_left  =  0.5  * (msg.axes[Axis.LEFT_TRIGGER] - AXIS_DEFAULTS[Axis.LEFT_TRIGGER])
                twist_msg.twist.linear.x = lin_x_right + lin_x_left

                # add some margin for joy stick
                twist_msg.twist.linear.y = msg.axes[Axis.RIGHT_STICK_X] if abs(msg.axes[Axis.RIGHT_STICK_X]) > 0.2 else 0.0
                twist_msg.twist.linear.z = msg.axes[Axis.RIGHT_STICK_Y] if abs(msg.axes[Axis.RIGHT_STICK_Y]) > 0.2 else 0.0

                twist_msg.twist.angular.x = msg.axes[Axis.LEFT_STICK_X] if abs(msg.axes[Axis.LEFT_STICK_X]) > 0.2 else 0.0
                twist_msg.twist.angular.y = msg.axes[Axis.LEFT_STICK_Y] if abs(msg.axes[Axis.LEFT_STICK_Y]) > 0.2 else 0.0

                roll_positive = msg.buttons[Button.RIGHT_BUMPER]
                roll_negative = -1 * (msg.buttons[Button.LEFT_BUMPER])
                twist_msg.twist.angular.z = float(roll_positive + roll_negative)

                if not self.isSim:
                    twist_msg.twist.linear.x *= MAX_SPEED_MUL
                    twist_msg.twist.linear.y *= MAX_SPEED_MUL
                    twist_msg.twist.linear.z *= MAX_SPEED_MUL
                    twist_msg.twist.angular.x *= MAX_SPEED_MUL
                    twist_msg.twist.angular.y *= MAX_SPEED_MUL
                    twist_msg.twist.angular.z *= MAX_SPEED_MUL

                publish_twist = True
        
        if publish_twist:
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = self.frame_to_publish
            self.twist_pub.publish(twist_msg)
        elif publish_joint:
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.header.frame_id = BASE_FRAME_ID
            self.joint_pub.publish(joint_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToServoPub()
    
    def signal_handler(sig, frame):
        node.close()
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # rclpy.spin(node)
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)  # Adjust the timeout as needed
    except KeyboardInterrupt:
        pass
    finally:
        signal_handler(signal.SIGINT, None)
    
if __name__ == '__main__':
    main()