import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import tf2_ros
from lidar_plane_filtering.tf2_sensor_msgs import do_transform_cloud


class LidarTransformer(Node):
    def __init__(self):
        super().__init__('lidar_transformer')

        # Create a rclpy.Duration object with a reasonable cache time
        cache_time = rclpy.duration.Duration(seconds=2.0)

        self.tf_buffer = tf2_ros.Buffer(cache_time)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/cloud_unstructured_fullframe',  # replace with your actual lidar topic
            self.lidar_callback,
            10
        )

        self.lidar_pub = self.create_publisher(
            PointCloud2,
            'lidar_world_topic',  # replace with your desired output topic
            10
        )

    def lidar_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                'world',  # target frame
                msg.header.frame_id,  # source frame
                rclpy.time.Time())
            transformed_cloud = do_transform_cloud(msg, transform)
            self.lidar_pub.publish(transformed_cloud)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Transform lookup failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LidarTransformer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
