import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class TrajectoryActionClient(Node):

    def __init__(self):

        super().__init__('points_publisher_node_action_client')
        self.action_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

    def send_goal(self):

        points = []

        point1_msg = JointTrajectoryPoint()
        point1_msg.positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        point1_msg.time_from_start = Duration(seconds=0.5).to_msg()

        point2_msg = JointTrajectoryPoint()
        point2_msg.positions = [0.1, 1.2, 0.3, -0.2, -0.3]
        point2_msg.time_from_start = Duration(
            seconds=1.0, nanoseconds=0).to_msg()

        point3_msg = JointTrajectoryPoint()
        point3_msg.positions = [0.1, 1.8, 0.3, -0.2, -0.6]
        point3_msg.time_from_start = Duration(
            seconds=1.2, nanoseconds=0).to_msg()

        point4_msg = JointTrajectoryPoint()
        point4_msg.positions = [0.1, 1.0, 0.3, -0.2, -0.3]
        point4_msg.time_from_start = Duration(
            seconds=1.4, nanoseconds=0).to_msg()

        point5_msg = JointTrajectoryPoint()
        point5_msg.positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        point5_msg.time_from_start = Duration(
            seconds=2.0, nanoseconds=0).to_msg()

        points.append(point1_msg)
        points.append(point2_msg)
        points.append(point3_msg)
        points.append(point4_msg)
        points.append(point5_msg)

        joint_names = ['prismatic_joint',
                       'joint1', 'joint2', 'joint3', 'joint4']
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.goal_time_tolerance = Duration(
            seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected ')
            return

        self.get_logger().info('Goal accepted')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        result = future.result().result
        self.get_logger().info('Result: '+str(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback


def main(args=None):
    i = 0
    while(i < 1000):
        rclpy.init()
        action_client = TrajectoryActionClient()
        future = action_client.send_goal()
        rclpy.spin(action_client)
        i = i+1


if __name__ == '__main__':
    main()
