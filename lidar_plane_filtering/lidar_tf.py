import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformListener, Buffer, TransformStamped
import numpy as np

class LidarTransformer(Node):
    def __init__(self):
        super().__init__('lidar_transformer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lidar_sub = self.create_subscription(PointCloud2, '/cloud_unstructured_fullframe', self.lidar_callback, 10)
        
        # Create a publisher for the transformed point cloud
        self.publisher = self.create_publisher(PointCloud2, '/transformed_lidar', 10)

    def lidar_callback(self, msg):
        try:
            # Get the most recent transform time
            most_recent_transform_time = self.tf_buffer.get_latest_common_time('lidar_link', 'world')

            # Use the most recent transform time as the current time
            if most_recent_transform_time.to_msg().sec != 0:
                current_time = most_recent_transform_time.to_msg()
            else:
                current_time = self.get_clock().now().to_msg()

            # Check time synchronization
            transform_time = self.tf_buffer.lookup_transform('world', 'lidar_link', current_time)
            time_diff = (current_time.sec - transform_time.header.stamp.sec) + (current_time.nanosec - transform_time.header.stamp.nanosec) * 1e-9
            self.get_logger().info(f"Time difference: {time_diff}")

            # Extract translation and rotation from TransformStamped
            translation = np.array([
                transform_time.transform.translation.x,
                transform_time.transform.translation.y,
                transform_time.transform.translation.z
            ])
            rotation = np.array([
                transform_time.transform.rotation.x,
                transform_time.transform.rotation.y,
                transform_time.transform.rotation.z,
                transform_time.transform.rotation.w
            ])

            # Create transformation matrix
            transformation_matrix = self.compose_transformation_matrix(translation, rotation)

            # Transform lidar data from lidar frame to world frame
            transformed_data = self.transform_point_cloud(msg, transformation_matrix)
            
            # Publish the transformed point cloud on a new topic
            self.publisher.publish(transformed_data)
            
        except Exception as e:
            self.get_logger().warn(f"Failed to transform lidar data: {str(e)}")

    def compose_transformation_matrix(self, translation, rotation):
        translation_matrix = np.eye(4)
        translation_matrix[:3, 3] = translation

        rotation_matrix = self.quaternion_to_matrix(rotation)

        transformation_matrix = np.dot(translation_matrix, rotation_matrix)
        return transformation_matrix

    def quaternion_to_matrix(self, q):
        q = np.array(q)
        n = np.dot(q, q)
        if n < np.finfo(q.dtype).eps:
            return np.eye(4)
        q *= np.sqrt(2.0 / n)
        q = np.outer(q, q)
        q *= 2.0
        q = np.array([
            [1.0 - q[2, 2] - q[3, 3], q[1, 2] - q[3, 0], q[1, 3] + q[2, 0], 0.0],
            [q[1, 2] + q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] - q[1, 0], 0.0],
            [q[1, 3] - q[2, 0], q[2, 3] + q[1, 0], 1.0 - q[1, 1] - q[2, 2], 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
        return q

    def transform_point_cloud(self, original_pc, transformation_matrix):
        
        # Apply the transformation matrix to the original lidar data
        transformed_points = []

        # Iterate over the original points and perform the transformation
        for i in range(0, len(original_pc.data), original_pc.point_step):
            # Extract x, y, z values from the original lidar data
            x = original_pc.data[i + 0] | (original_pc.data[i + 1] << 8) | (original_pc.data[i + 2] << 16) | (original_pc.data[i + 3] << 24)
            y = original_pc.data[i + 4] | (original_pc.data[i + 5] << 8) | (original_pc.data[i + 6] << 16) | (original_pc.data[i + 7] << 24)
            z = original_pc.data[i + 8] | (original_pc.data[i + 9] << 8) | (original_pc.data[i + 10] << 16) | (original_pc.data[i + 11] << 24)

            # Convert the values to integers
            x, y, z = int(x), int(y), int(z)

            # Example: Apply the transformation (replace this with your actual transformation logic)
            transformed_x = x + 1  # Replace with your transformation for x coordinate
            transformed_y = y + 1  # Replace with your transformation for y coordinate
            transformed_z = z + 1  # Replace with your transformation for z coordinate

            # Append the transformed point to the list
            transformed_points.extend([
                transformed_x & 0xFF, (transformed_x >> 8) & 0xFF, (transformed_x >> 16) & 0xFF, (transformed_x >> 24) & 0xFF,
                transformed_y & 0xFF, (transformed_y >> 8) & 0xFF, (transformed_y >> 16) & 0xFF, (transformed_y >> 24) & 0xFF,
                transformed_z & 0xFF, (transformed_z >> 8) & 0xFF, (transformed_z >> 16) & 0xFF, (transformed_z >> 24) & 0xFF,
                original_pc.data[i + 12], original_pc.data[i + 13], original_pc.data[i + 14], original_pc.data[i + 15]
            ])

        # Continue with the rest of your code


        # Create a new PointCloud2 message for the transformed data
        transformed_pc = PointCloud2()
        transformed_pc.header = original_pc.header
        transformed_pc.height = 1
        transformed_pc.width = len(transformed_points)
        transformed_pc.fields = original_pc.fields
        transformed_pc.is_bigendian = False
        transformed_pc.point_step = 16  # Assuming XYZ data format
        transformed_pc.row_step = transformed_pc.point_step * transformed_pc.width
        transformed_pc.is_dense = False
        transformed_pc.data = bytearray(transformed_points)

        return transformed_pc

def main():
    rclpy.init()
    node = LidarTransformer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
