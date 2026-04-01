import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import math

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.path = None
        self.current_pose = None
        self.create_timer(0.1, self.control_loop)

    def path_cb(self, msg):
        self.path = msg.poses

    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose

    def get_yaw_from_quaternion(self, q):
        # Convert quaternion to yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if self.path is None or self.current_pose is None:
            return
        
        # 1. Get Target (End of path)
        target = self.path[-1].pose.position 
        curr_x = self.current_pose.position.x
        curr_y = self.current_pose.position.y
        
        # 2. Get Current Yaw
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        # 3. Calculate distance and angle to target in Global Frame
        dist_to_goal = math.hypot(target.x - curr_x, target.y - curr_y)
        global_angle = math.atan2(target.y - curr_y, target.x - curr_x)
        
        # 4. Calculate steering error (Angle difference)
        steering_error = global_angle - current_yaw
        
        # Normalize angle to [-pi, pi] so the robot doesn't spin 360 degrees
        while steering_error > math.pi: steering_error -= 2.0 * math.pi
        while steering_error < -math.pi: steering_error += 2.0 * math.pi
        
        twist = Twist()
        if dist_to_goal < 0.3:
            self.get_logger().info("Goal Reached!")
            self.path = None 
            self.cmd_pub.publish(twist) # Send 0.0 to stop
        else:
            # 1. Higher Base Speed
            # Increased from 0.2 to 0.5 (Adjust this based on your robot's limits)
            base_linear_speed = 0.5 
            
            # 2. Dynamic Speed Scaling
            # This slows the robot down ONLY if it needs to make a sharp turn.
            # If steering_error is small (straight line), it goes full speed.
            twist.linear.x = base_linear_speed * max(0.1, 1.0 - (abs(steering_error) / math.pi))
            
            # 3. Aggressive Steering
            # Increased gain from 0.8 to 1.5 so it reacts faster at high speeds.
            # If it starts wobbling/oscillating, lower this to 1.2.
            twist.angular.z = steering_error * 1.5 
            
            self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    rclpy.spin(PurePursuit())
    rclpy.shutdown()