#This nodes suscribes to isaac sims topic where the ik is solved (joint_states_response) and publishes the joint angular position 
#into the rviz ur5e digital twin.
#ROS2 brigde for isaac sim and real and fake UR5e robot

import rclpy
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint 
from sensor_msgs.msg import JointState



class Controller_ik(Node):
    def __init__(self):
        super().__init__("controller_ik")
        self.suscription = self.create_subscription(JointState, '/joint_states_response', self.joint_state_callback, 10) #topic where isaac sim publishes solved ik
        self.publisher = self.create_publisher(JointTrajectory, 'scaled_joint_trajectory_controller/joint_trajectory', 10) #publishes new positon for rviz digital twin 
        self.joint_names = ['shoulder_pan_joint' ,'shoulder_lift_joint' ,'elbow_joint' ,'wrist_1_joint' ,'wrist_2_joint' ,'wrist_3_joint']
        self.joints_positions = None
        #self.timer = self.create_timer(8.0, self.timer_callback) #long timer so traj can get enough time to complete

        self.get_logger().info('Node successfully initialiized!!!')

    def joint_state_callback(self, msg):
        self.joints_positions = msg.position
        self.send_controller(self.joints_positions, 1)


    def send_controller(self, positions, duration): #method to send traj and publish mgs
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = duration

        msg.points = [point]
        self.publisher.publish(msg)
        self.get_logger().info(f'Actual positions:{point.positions}')


def main(args = None):
    rclpy.init(args = args)
    node = Controller_ik()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
