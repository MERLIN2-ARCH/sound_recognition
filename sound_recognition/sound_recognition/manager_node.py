""" ROS2 Node for Dialog Manger """

import time
import rclpy
from std_srvs.srv import Empty
from std_msgs.msg import String
from rclpy.action import ActionServer
from sound_recognition_msgs.action import Listen
from simple_node import Node


class ManagerNode(Node):
    """ Manager Node Class """

    def __init__(self):
        super().__init__("manager_node")

        self.doorbell = False

        # service clients
        self.__start_listening_client = self.create_client(
            Empty, "start_ad_listening")
        self.__stop_listening_client = self.create_client(
            Empty, "stop_ad_listening")

        # sub
        """ self.__subscription = self.create_subscription(
            String,
            "sound_recognition",
            self.__ad_callback,
            10) """

        # action server
        self.__action_server = self.create_action_server(
            Listen,
            "listen_doorbell",
            execute_callback=self.__execute_server)

    def __ad_callback(self, msg):
        """ callback

        Args:
            msg (String): sound class recognized
        """

        self.get_logger().info("Manager: " + str(msg.data))
        # self.__pub.publish(msg)

        if not self.doorbell:
            self.get_logger().info("doorbell")
            #self.doorbell = True

    def start_ad(self):
        """ start ad method """

        req = Empty.Request()
        self.__start_listening_client.wait_for_service()
        self.__start_listening_client.call(req)
        self.get_logger().info("starting sound_recognition")

    def stop_ad(self):
        """ stop ad method """

        req = Empty.Request()
        self.__stop_listening_client.wait_for_service()
        self.__stop_listening_client.call(req)
        self.get_logger().info("stopping sound_recognition")

    def __execute_server(self, goal_handle) -> Listen.Result:
        """ action server execute callback

        Args:
            goal_handle ([type]): goal_handle

        Returns:
            Listen.Result: action server result (boolean doorbell detection)
        """

        self.doorbell = False

        # starting sound_recognition
        self.start_ad()

        # wait for message
        while(not self.doorbell):
            self.get_logger().info("Waiting for doorbell detection.")
            time.sleep(0.5)

        # stoping sound_recognition
        self.stop_ad()

        # results
        result = Listen.Result()

        ''' if self.__action_server.is_canceled():
            self.__action_server.wait_for_canceling()
            goal_handle.canceled() '''

        # else:
        result.doorbell_detection = self.doorbell
        goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)

    node = ManagerNode()

    node.join_spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
