import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from shared_custom_interfaces.actions import RoomNavigation


class KarlNavigator(Node):
    def __init__(self):
        super().__init__(node_name='karl_navigator')
        self.navigator = TurtleBot4Navigator()
        self._action_server = ActionServer(
            self,
            RoomNavigation,
            'nav2room',
            self.execute_callback)
        # TODO: set initial pose

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # feedback_msg = RoomNavigation.Feedback()
        feedback_msg = self.navigator.getFeedback()

        self.start_to_room(goal_handle.request.room)

        # TODO: log and publish feedback

        goal_handle.succeed()
        # result = RoomNavigation.Result()
        result = self.navigator.getResult()
        return result

    def start_to_room(self, room: str):
        # TODO: check docking status and undock if necessary

        # TODO: determine pose from room input

        self.navigator.goToPose(pose)


def main():
    rclpy.init()

    navigator = KarlNavigator()

    rclpy.spin(navigator)

    navigator.destroy_node()
    rclpy.shutdown()


# def main():
#     rclpy.init()
#
#     navigator = TurtleBot4Navigator()
#
#     # Start on dock
#     if not navigator.getDockedStatus():
#         navigator.info('Docking before intialising pose')
#         navigator.dock()
#
#     # Set initial pose
#     initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
#     navigator.setInitialPose(initial_pose)
#
#     # Wait for Nav2
#     navigator.waitUntilNav2Active()
#
#     # Set goal poses
#     goal_pose = navigator.getPoseStamped([-13.0, 9.0], TurtleBot4Directions.EAST)
#
#     # Undock
#     navigator.undock()
#
#     # Go to each goal pose
#     navigator.startToPose(goal_pose)
#
#     rclpy.shutdown()


if __name__ == '__main__':
    main()
