import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Imu

class MPUTfPublisher(Node):
    def __init__(self):
        super().__init__('mpu_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)

        self.imu_orientation = None
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10)
        self.subscription  # prevent unused variable warning

    def imu_callback(self, msg):
        # Extract IMU orientation
        self.imu_orientation = msg.orientation
        self.publish_tf()

    def publish_tf(self):
        if self.imu_orientation:
            # Compute translation and rotation based on IMU data
            translation = (0.0, 0.0, 0.0)  # Replace with your translation data
            rotation = Quaternion(
                x=self.imu_orientation.x,
                y=self.imu_orientation.y,
                z=self.imu_orientation.z,
                w=self.imu_orientation.w
            )

            # Create TransformStamped message
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'world'  # The world frame
            transform.child_frame_id = 'base_link'
            transform.transform.translation.x = translation[0]
            transform.transform.translation.y = translation[1]
            transform.transform.translation.z = translation[2]
            transform.transform.rotation = rotation

            # Publish the transformation
            self.tf_broadcaster.sendTransform(transform)
            self.get_logger().info('Transform published')
            self.get_logger().info(f'Translation: {translation}, Rotation: {rotation}')

def main(args=None):
    rclpy.init(args=args)
    mpu_tf_publisher = MPUTfPublisher()
    rclpy.spin(mpu_tf_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()