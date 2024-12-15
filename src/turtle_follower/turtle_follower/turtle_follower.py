import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Pose
from math import pow, atan2, sqrt


class TurtleMover(Node):
   def __init__(self):
      super().__init__("turtle_follower")
      self.publisher_ = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
      self.subscriber_two = self.create_subscription(Pose, '/turtle2/pose', self.reader_callback, 10) 
      self.subscriber_one = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
      
      self.pose = Pose()
      self.deadzone = 0.01

   def update_pose(self, data):
      self.pose = data
      self.pose.x = round(self.pose.x, 4)
      self.pose.y = round(self.pose.y, 4)
            
   def euclidean_distance(self, x_pos, y_pos):
      distance = sqrt(pow(self.pose.x - x_pos, 2) + pow(self.pose.y - y_pos, 2))
      return distance if distance > self.deadzone else 0
   
   def rotational_error(self, x_pos, y_pos, yaw):
      return atan2(self.pose.y - y_pos, self.pose.x - x_pos) - yaw
   
   def linear_velocity(self, error, kp=0.5):
      return error * kp
   
   def angular_velocity(self, error, kp=6):
      return error * kp
   
   def reader_callback(self, msg):
      x_pos = msg.x
      y_pos = msg.y
      yaw = msg.theta
      
      distance_error = self.euclidean_distance(x_pos, y_pos)
      rotational_error = self.rotational_error(x_pos, y_pos, yaw)
      
      vel_msg = Twist()
      vel_msg.linear.x = self.linear_velocity(distance_error, 0.5)
      vel_msg.angular.z = self.angular_velocity(rotational_error, 6)
      self.publisher_.publish(vel_msg)
      
        
def main(args=None):
   rclpy.init(args=args)
   node = TurtleMover()
   rclpy.spin(node)
   rclpy.shutdown()

if __name__ == '__main__':
    main()