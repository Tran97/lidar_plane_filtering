import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class LidarFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_filter_node')
        self.lidar_subscription = self.create_subscription(
            PointCloud2,
            '/cloud_to_world',  # Replace with your actual lidar topic
            self.filter_lidar_callback,
            10  # Adjust the queue size according to your requirements
        )

        self.publisher = self.create_publisher(
            PointCloud2,
            'plane_filtered_cloud',  # Replace with your desired output topic
            10
        )
        self.vertical_threshold = 1  # Adjust the vertical threshold as needed
        self.imu_orientation = None

    def filter_lidar_callback(self, msg):
        # Extract point cloud data
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = list(points)

        # Filter points based on vertical distance and IMU orientation
        filtered_points = [
            point for point in points
            if point[2] > self.vertical_threshold
        ]

        # Create a new PointCloud2 message
        new_msg = PointCloud2()
        new_msg.header = msg.header
        new_msg.height = 1
        new_msg.width = len(filtered_points)
        new_msg.fields = msg.fields
        new_msg.is_bigendian = False
        new_msg.point_step = 16  # Assuming XYZ data format
        new_msg.row_step = new_msg.point_step * new_msg.width
        new_msg.is_dense = False

        # Convert filtered_points directly to bytes
        new_msg.data = np.array(filtered_points).tobytes()

        # Publish the filtered point cloud
        self.publisher.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    lidar_filter_node = LidarFilterNode()
    rclpy.spin(lidar_filter_node)
    lidar_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()