import time, os, math
import rclpy
import pygame
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty


class JoyTeleopNode(Node):
    def __init__(self):
        super().__init__('joy_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.holding_heading = False
        self.power = False

        self.spd = 0.25
        self.prev_pressed = False
        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        self.stop_motor_srv = self.create_client(Empty, 'stop_motor')
        self.start_motor_srv = self.create_client(Empty, 'start_motor')

        self.hold_srv = self.create_client(Empty, 'hold_heading')

        self.deadzone = 0.1
        self.last_motion = 0

        #self.x_s, self.y_s, self.th_s = 0, 0, 0

    def get_joystick_axis(self, axis):
        value = self.js.get_axis(axis)
        d_value = value - self.deadzone if value > self.deadzone else (value + self.deadzone if value < -self.deadzone else 0)
        #print(value, d_value, self.deadzone)
        return d_value
    
    def timer_callback(self):
        self.get_logger().info("Started!", once=True)
        pygame.event.pump()

        x_spd = -self.get_joystick_axis(1) * 0.05
        y_spd = -self.get_joystick_axis(0) * 0.05
        th_spd = -math.copysign(self.get_joystick_axis(2)**2, self.get_joystick_axis(2)) * 0.5

        speed_increase = 1 + (self.js.get_axis(5) + 1)**3 * 10
        speed_decrease = 1 / (1 + (1 + self.js.get_axis(2)) * 2.5)

        x_spd *= speed_decrease * speed_increase
        y_spd *= speed_decrease * speed_increase

        if self.js.get_button(9) and not self.prev_pressed:
            self.get_logger().info("Holding!")
            self.hold_srv.call_async(Empty.Request())


        if self.js.get_button(7) and not self.prev_pressed:
            self.get_logger().info("Toggle LiDAR!")

            if self.power:
                self.get_logger().info("Stop LiDAR!")
                self.stop_motor_srv.call_async(Empty.Request())
            else:
                self.get_logger().info("Start LiDAR!")
                self.start_motor_srv.call_async(Empty.Request())

            self.prev_pressed = True
            self.power = not self.power

        self.prev_pressed = self.js.get_button(9) or self.js.get_button(7)

        self.get_logger().info(f"x: {x_spd} m/s, y: {y_spd} m/s, th: {th_spd} rad/s", throttle_duration_sec=1)

        msg = Twist()

        msg.linear.x = float(x_spd)
        msg.linear.y = float(y_spd)

        msg.angular.z = float(th_spd)

        if msg != Twist():
            self.last_motion = time.time()
        
        if time.time() - self.last_motion < 2:
            self.publisher_.publish(msg)


def main(args=None):
    #os.environ["SDL_VIDEODRIVER"] = "dummy"
    pygame.display.init()

    pygame.joystick.init()

    rclpy.init(args=args)

    my_node = JoyTeleopNode()

    rclpy.spin(my_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()