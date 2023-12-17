#import rclpy
#from rclpy.node import Node
#from sensor_msgs.msg import PointCloud2
#import sensor_msgs_py.point_cloud2 as pc2
#import numpy as np

#class LidarFilterNode(Node):
    #def __init__(self):
        #super().__init__('lidar_filter_node')
        #self.subscription = self.create_subscription(
            #PointCloud2,
            #'/cloud_unstructured_fullframe',  # Replace with your actual lidar topic
            #self.filter_lidar_callback,
            #10  # Adjust the queue size according to your requirements
        #)
        #self.publisher = self.create_publisher(
            #PointCloud2,
            #'output_lidar_topic',  # Replace with your desired output topic
            #10
        #)
        #self.vertical_threshold = -1  # Adjust the vertical threshold as needed

    #def filter_lidar_callback(self, msg):
        ## Extract point cloud data
        #points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        #points = list(points)

        ## Filter points based on vertical distance
        #filtered_points = [point for point in points if point[2] > self.vertical_threshold]

        ## Create a new PointCloud2 message
        #new_msg = PointCloud2()
        #new_msg.header = msg.header
        #new_msg.height = 1
        #new_msg.width = len(filtered_points)
        #new_msg.fields = msg.fields
        #new_msg.is_bigendian = False
        #new_msg.point_step = 16  # Assuming XYZ data format
        #new_msg.row_step = new_msg.point_step * new_msg.width
        #new_msg.is_dense = False

        ## Convert filtered_points directly to bytes
        #new_msg.data = np.array(filtered_points).tobytes()

        ## Publish the filtered point cloud
        #self.publisher.publish(new_msg)

#def main(args=None):
    #rclpy.init(args=args)
    #lidar_filter_node = LidarFilterNode()
    #rclpy.spin(lidar_filter_node)
    #lidar_filter_node.destroy_node()
    #rclpy.shutdown()

#if __name__ == '__main__':
    #main()


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from scipy.spatial.transform import Rotation

class LidarFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_filter_node')
        self.lidar_subscription = self.create_subscription(
            PointCloud2,
            '/cloud_unstructured_fullframe',  # Replace with your actual lidar topic
            self.filter_lidar_callback,
            10  # Adjust the queue size according to your requirements
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',  # Replace with your actual IMU topic
            self.imu_callback,
            10
        )
        self.publisher = self.create_publisher(
            PointCloud2,
            'output_lidar_topic',  # Replace with your desired output topic
            10
        )
        self.vertical_threshold = -0.2  # Adjust the vertical threshold as needed
        self.imu_orientation = None

    def filter_lidar_callback(self, msg):
        # Extract point cloud data
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = list(points)

        # Filter points based on vertical distance and IMU orientation
        filtered_points = [
            point for point in points
            if point[2] > self.vertical_threshold and self.filter_by_imu_orientation(point)
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

    def imu_callback(self, msg):
        # Extract IMU orientation
        self.imu_orientation = msg.orientation

    def filter_by_imu_orientation(self, point):
        # Implement your logic to filter points based on IMU orientation if needed
        # You can use self.imu_orientation in this method
        # Return True if the point should be kept, False otherwise
        return True

def main(args=None):
    rclpy.init(args=args)
    lidar_filter_node = LidarFilterNode()
    rclpy.spin(lidar_filter_node)
    lidar_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    


#import rclpy
#from rclpy.node import Node
#from sensor_msgs.msg import PointCloud2
#from geometry_msgs.msg import TransformStamped
#import tf2_ros
#import sensor_msgs_py.point_cloud2 as pc2
#import numpy as np

#class LidarFilterNode(Node):
    #def __init__(self):
        #super().__init__('lidar_filter_node')
        #self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        #self.subscription = self.create_subscription(
            #PointCloud2,
            #'/cloud_unstructured_fullframe',  # Replace with your actual lidar topic
            #self.lidar_callback,
            #10  # Adjust the queue size according to your requirements
        #)
        #self.publisher = self.create_publisher(
            #PointCloud2,
            #'output_lidar_topic',  # Replace with your desired output topic
            #10
        #)

    #def lidar_callback(self, msg):
        ## Broadcast lidar to world transform
        #lidar_transform = TransformStamped()
        #lidar_transform.header.stamp = self.get_clock().now().to_msg()
        #lidar_transform.header.frame_id = msg.header.frame_id  # Source frame (lidar frame)
        #lidar_transform.child_frame_id = 'world'  # Target frame
        ## Set the translation and rotation values based on your use case
        #lidar_transform.transform.translation.x = 0.0
        #lidar_transform.transform.translation.y = 0.0
        #lidar_transform.transform.translation.z = 0.0
        #lidar_transform.transform.rotation.x = 0.0
        #lidar_transform.transform.rotation.y = 0.0
        #lidar_transform.transform.rotation.z = 0.0
        #lidar_transform.transform.rotation.w = 1.0

        ## Broadcast lidar to world transform
        #self.tf_broadcaster.sendTransform(lidar_transform)

        ## Rest of the lidar_callback method...
        ## Extract point cloud data
        #points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        #points = list(points)

        ## Filter out half of the points
        #filtered_points = points[:len(points)//2]

        ## Create a new PointCloud2 message
        #new_msg = PointCloud2()
        #new_msg.header = msg.header
        #new_msg.height = 1
        #new_msg.width = len(filtered_points)
        #new_msg.fields = msg.fields
        #new_msg.is_bigendian = False
        #new_msg.point_step = 16  # Assuming XYZ data format
        #new_msg.row_step = new_msg.point_step * new_msg.width
        #new_msg.is_dense = False
        #new_msg.data = np.array(filtered_points, dtype=np.float32).tobytes()

        ## Publish the filtered point cloud
        #self.publisher.publish(new_msg)

#def main(args=None):
    #rclpy.init(args=args)
    #lidar_filter_node = LidarFilterNode()
    #rclpy.spin(lidar_filter_node)
    #lidar_filter_node.destroy_node()
    #rclpy.shutdown()

#if __name__ == '__main__':
    #main()
