import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from shared_custom_interfaces.action import RoomNavigation


class KarlNavigator(Node):
    def __init__(self):
        super().__init__(node_name='karl_navigator')
        self.navigator = TurtleBot4Navigator()
        self._action_server = ActionServer(
            self,
            RoomNavigation,
            'navToRoom',
            self.execute_callback)
        # TODO: set initial pose
        initial_pose = self.navigator.getPoseStamped([-0.86, -0.87], TurtleBot4Directions.SOUTH)
        self.navigator.setInitialPose(initial_pose)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # feedback_msg = RoomNavigation.Feedback()
        feedback_msg = self.navigator.getFeedback()

        self.navigator.waitUntilNav2Active()
        self.start_to_room(goal_handle.request.room)

        # TODO: log and publish feedback

        goal_handle.succeed()
        result = self.navigator.getResult()
        return result

    def start_to_room(self, room: str):
        # TODO: check docking status and undock if necessary

        # TODO: determine pose from room input
        pose = {
            "Home": self.navigator.getPoseStamped([-0.86, -0.87], TurtleBot4Directions.SOUTH),
            "Odyssey": self.navigator.getPoseStamped([-0.2, 9.66], TurtleBot4Directions.SOUTH),
            "Sojourner": self.navigator.getPoseStamped([-7.29, 8.53], TurtleBot4Directions.SOUTH),
            "Curiosity": self.navigator.getPoseStamped([-1.59, -0.66], TurtleBot4Directions.SOUTH),
            "Viking": self.navigator.getPoseStamped([-12.03, 8.04], TurtleBot4Directions.SOUTH),
            "Opportunity": self.navigator.getPoseStamped([-2.03, 0.97], TurtleBot4Directions.SOUTH)
        }

        self.navigator.startToPose(pose[room])

        if room == "Home":
            if self.navigator.getDockedStatus():
                self.navigator.dock()



def main():
    rclpy.init()

    navigator = KarlNavigator()

    rclpy.spin(navigator)

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
