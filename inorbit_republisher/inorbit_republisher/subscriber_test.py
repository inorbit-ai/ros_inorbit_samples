import rclpy
import pytest
from std_msgs.msg import String
from time import sleep


class TestRepublisher:


    def subscription_callback(self, msg):
        self.received_message = msg.data

    def setup_method(self):
        rclpy.init()
    #Basically links my subscriber to the republisher topic in republisher.py
        # Create a test node for subscribing to republished_topic
        self.node = rclpy.create_node('test_republisher')

        # Create a subscription to 'republished_topic'
        self.test_subscription = self.node.create_subscription(
            String,
            'republished_topic',
            self.subscription_callback,
            10
        )

        # Create a publisher for the 'input_topic'
        self.test_publisher = self.node.create_publisher(String, 'input_topic', 10)

    def teardown_method(self):
        # Clean up the node
        self.node.destroy_node()
        rclpy.shutdown()

    def test_republish_message(self):
        # Send a test message to the 'input_topic'
        test_msg = String()
        test_msg.data = 'Eoo'
        self.test_publisher.publish(test_msg)

        # Give time for the callback to trigger and republish the message
        sleep(1)

        # Process one spin to simulate receiving the message and the timer callback
        rclpy.spin_once(self.node)

        # Assert that the received message is the same as the published one
        assert self.received_message == 'Eoo', \
            f"Expected 'Eoo', but got {self.received_message}"

        # Log success
        print("Daylight come and me wan' go home")

