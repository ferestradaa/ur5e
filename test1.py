#this code manually sends a random trajectory to ur5e robot to confirm communication
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint 
import time

class ur52Control(Node):
    def __init__(self):
        super().__init__('kinova_coomander')
        self.publisher = self.create_publisher(JointTrajectory, 'scaled_joint_trajectory_controller/joint_trajectory' ,10) #topic for joint position
        self.joint_names = ['shoulder_pan_joint' ,'shoulder_lift_joint' ,'elbow_joint' ,'wrist_1_joint' ,'wrist_2_joint' ,'wrist_3_joint']

        #self.startup_timer = self.create_timer(3.0, self.startup_init)
        self.timer = self.create_timer(8.0, self.timer_callback) #long timer so traj can get enough time to complete
        self.step = 0
        self.get_logger().info('Node successfully initialiized!')

    
    def startup_init(self): #initial condition (ignore)
        self.send_controller([0.0]*6, 5)
        self.startup_timer.cancel()


    def timer_callback(self):
        if self.step == 0:
            self.send_controller([0.5, -2.3,-2.4,-1.7, 0.1, 0.0], 7 #sending joints position (rads) and duration time)
            self.step +=1
            self.get_logger().info('sending traj')
        elif self.step == 1:
            self.send_controller([0.0]*6, 7) #get back to origin after completing the task
        
            self.get_logger().info('back to zero')

    def send_controller(self, positions, duration): #method to send traj and publish mgs
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = duration

        msg.points = [point]
        self.publisher.publish(msg)

def main(args = None):
    rclpy.init(args = args)
    node = ur5eControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
