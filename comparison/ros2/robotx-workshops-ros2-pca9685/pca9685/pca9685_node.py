import gc
from pca9685.models.config import PCA9685Config
from pca9685.services.pca9685 import PCA9685
from py_gap_follower.gc import GcConfig, GcThresholds
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class PCA9685SubscriberNode(Node):
    def __init__(self):
        super().__init__("pca9685_node")

        # Declare parameters with default values
        self.declare_parameter("bus", 1)
        self.declare_parameter("address", 0x40)
        self.declare_parameter("frequency", 60)

        # Declare Garbabge Collector parameters
        self.declare_parameter("gc.disable", False)
        self.declare_parameter("gc.thresholds.gen_0", 700)
        self.declare_parameter("gc.thresholds.gen_1", 10)
        self.declare_parameter("gc.thresholds.gen_2", 10)

        # Retrieve parameter values
        bus = self.get_parameter("bus").get_parameter_value().integer_value
        address = self.get_parameter("address").get_parameter_value().integer_value
        frequency = self.get_parameter("frequency").get_parameter_value().integer_value

        config = PCA9685Config(busnum=bus, address=address, frequency=frequency)
        self.pca9685 = PCA9685(config=config)

        # Create a dictionary to hold subscribers
        self.channel_subscriptions = {}

        # Create a subscriber for each channel
        number_of_channels = 16  # PCA9685 always has 16 channels
        for i in range(number_of_channels):
            topic_name = f"/pwm_channel_{i}"
            self.channel_subscriptions[i] = self.create_subscription(
                Int32,
                topic_name,
                lambda msg, channel=i: self.channel_listener_callback(msg, channel),  # type: ignore
                10,  # Queue size
            )
        self.get_logger().info(f"Subscribing to {number_of_channels} if channels")

        gc_config = GcConfig(
            disable=self.get_parameter("gc.disable").get_parameter_value().bool_value,
            thresholds=GcThresholds(
                gen_0=self.get_parameter("gc.thresholds.gen_0")
                .get_parameter_value()
                .integer_value,
                gen_1=self.get_parameter("gc.thresholds.gen_1")
                .get_parameter_value()
                .integer_value,
                gen_2=self.get_parameter("gc.thresholds.gen_2")
                .get_parameter_value()
                .integer_value,
            ),
        )
        if gc_config.disable:
            self.get_logger().info("Garbage collector disabled")
            gc.disable()
        else:
            self.get_logger().info("Garbage collector enabled")
            gc.enable()
            # Set the garbage collector thresholds
            gc.set_threshold(
                gc_config.thresholds.gen_0,
                gc_config.thresholds.gen_1,
                gc_config.thresholds.gen_2,
            )
            self.get_logger().info(
                f"Garbage collector thresholds set to {gc_config.thresholds}"
            )

    def channel_listener_callback(self, msg: Int32, channel: int):
        """
        Callback function for individual channel subscriptions.
        Receives a single Int32 message representing the pulse.
        """
        pulse = msg.data
        # Optional: Add validation for pulse value if needed (e.g., 0 to 4095)
        if not 0 <= pulse <= 4095:
            self.get_logger().warn(
                f"Received pulse value {pulse} out of typical range for channel {channel}. Setting anyway."
            )

        try:
            self.pca9685.set_pulse(channel, pulse)
            # self.get_logger().debug(f"Set pulse {pulse} for channel {channel}") # Uncomment for debugging
        except Exception as e:
            self.get_logger().error(f"Error setting pulse for channel {channel}: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = PCA9685SubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
