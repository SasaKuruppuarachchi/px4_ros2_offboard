#!/usr/bin/env python3
import sys
import geometry_msgs.msg
import rclpy
import std_msgs.msg
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rclpy.node import Node

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. 
Using the arrow keys and WASD you have Mode 2 RC controls.
W: Up
S: Down
A: Yaw Left
D: Yaw Right
Up Arrow: Pitch Forward
Down Arrow: Pitch Backward
Left Arrow: Roll Left
Right Arrow: Roll Right

Press SPACE to arm/disarm the drone
"""




class OffboardCommander(Node):
    def __init__(self):
        super().__init__('offboard_commander')
        self.settings = self.save_terminal_settings()
        self.speed = 0.5
        self.turn = 0.2
        self.x_val = 0.0
        self.y_val = 0.0
        self.z_val = 0.0
        self.yaw_val = 0.0
        self.arm_toggle = False
        self.joy_found = False
        self.old_joy_axes = Joy.axes
        self.old_joy_buttons = Joy.buttons
        self.joy_mode = True

        
        #self.node = rclpy.create_node('offboard_commander')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.moveBindings = {
            'w': (0, 0, 1, 0),  # Z+
            's': (0, 0, -1, 0),  # Z-
            'a': (0, 0, 0, -1),  # Yaw+
            'd': (0, 0, 0, 1),  # Yaw-
            '\x1b[A': (1, 0, 0, 0),  # Up Arrow
            '\x1b[B': (-1, 0, 0, 0),  # Down Arrow
            '\x1b[C': (0, -1, 0, 0),  # Right Arrow
            '\x1b[D': (0, 1, 0, 0),  # Left Arrow
        }
        self.subscription = self.create_subscription(Joy, '/joy', lambda msg: self.joy_listener_callback(msg, self.pub), 100)
        print(self.subscription)
        self.pub = self.create_publisher(
            geometry_msgs.msg.Twist,
            '/offboard_velocity_cmd',
            QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
            )
        )
        self.arm_pub = self.create_publisher(std_msgs.msg.Bool, '/arm_message', qos_profile)

    def save_terminal_settings(self):
        if sys.platform == 'win32':
            return None
        return termios.tcgetattr(sys.stdin)

    def restore_terminal_settings(self, old_settings):
        if sys.platform == 'win32':
            return
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def get_key(self):
        if sys.platform == 'win32':
            return msvcrt.getwch()
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        if key == '\x1b':  # Handle arrow keys
            additional_chars = sys.stdin.read(2)
            key += additional_chars
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            print(msg)
            while True:
                key = self.get_key()
                self.process_key(key)
                if key == '\x03':  # Handle Ctrl+C to exit
                    break
        except Exception as e:
            print(e)
        finally:
            self.stop_motion()
            self.restore_terminal_settings(self.settings)

    def process_key(self, key):
        if key in self.moveBindings:
            x, y, z, th = self.moveBindings[key]
        else:
            x = y = z = th = 0.0

        if key == ' ':
            self.arm_toggle = not self.arm_toggle
            arm_msg = std_msgs.msg.Bool()
            arm_msg.data = self.arm_toggle
            self.arm_pub.publish(arm_msg)
            print(f"Arm toggle is now: {self.arm_toggle}")

        self.publish_twist(x, y, z, th)
        
    def map_num(self,num,min,max):
        return min + (num + 1) * (max - min) / 2
    
    def joy_listener_callback(self, msg, publisher):
        #Extract joystick axes and buttons
        # >>> axes
        # 0 - yaw
        # 1 - Throttle
        # 2 - (L2)
        # 3 - Roll
        # 4 - Pitch
        # 5 - (R2)
        # 6 - Dpad x [-1,1]
        # 7 - Dpad y 
        
        # >>> Buttons
        # 0-(X), 1-(O), 2-(A), 3-(#), 4-(L1), 5-(R1),  11-(L3), 12-(R3),

        min_v_vel, max_v_vel = -5,5
        min_h_vel, max_h_vel = -2,2
        min_z_vel, max_z_vel = -3,3
        
        if not self.joy_found:
            self.joy_found = True
            print("Joystick recieved")
        
        
        if msg.buttons[4]:
            self.arm_toggle = True
            arm_msg = std_msgs.msg.Bool()
            arm_msg.data = self.arm_toggle
            self.arm_pub.publish(arm_msg)
            print(f"Arm toggle is now: {self.arm_toggle}")
        if msg.buttons[5]:
            self.arm_toggle = False
            self.stop_motion()
            arm_msg = std_msgs.msg.Bool()
            arm_msg.data = self.arm_toggle
            self.arm_pub.publish(arm_msg)
            print(f"Arm toggle is now: {self.arm_toggle}")
            
                #land
        
        if self.arm_toggle:
            yaw     =self.map_num(msg.axes[0],min_z_vel, max_z_vel)
            throttle=self.map_num(msg.axes[1],min_h_vel, max_h_vel)
            roll    =self.map_num(msg.axes[3],min_v_vel, max_v_vel)
            pitch   =self.map_num(msg.axes[4],min_v_vel, max_v_vel)

            #Create Twist message
            twist_msg = Twist()
            twist_msg.linear.x  = pitch
            twist_msg.linear.y  = roll
            twist_msg.angular.z = yaw
            twist_msg.linear.z  = throttle
            #Publish the Twist message
            if self.joy_mode:
                publisher.publish(twist_msg)
            self.old_joy_axes = msg.axes
            self.old_joy_buttons = msg.buttons

    def publish_twist(self, x, y, z, th):
        twist = geometry_msgs.msg.Twist()
        self.x_val += x * self.speed
        self.y_val += y * self.speed
        self.z_val += z * self.speed
        self.yaw_val += th * self.turn
        twist.linear.x = self.x_val
        twist.linear.y = self.y_val
        twist.linear.z = self.z_val
        twist.angular.z = self.yaw_val
        self.pub.publish(twist)
        print(f"X: {twist.linear.x} Y: {twist.linear.y} Z: {twist.linear.z} Yaw: {twist.angular.z}")

    def stop_motion(self):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = twist.linear.y = twist.linear.z = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)

def main(args=None):
    rclpy.init()
    print(msg)
    commander = OffboardCommander()
    if not commander.joy_mode:
        commander.run()
    rclpy.spin(commander)

    commander.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
