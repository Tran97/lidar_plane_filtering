import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuConverterNode(Node):

    def __init__(self):
        
        super().__init__('imu_converter_node')
        
        self.subscription = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
       
        
    def imu_callback(self, msg):
        msg.angular_velocity.x *= (math.pi / 180.0)
        msg.angular_velocity.y *= (math.pi / 180.0)
        msg.angular_velocity.z *= (math.pi / 180.0)
        msg.header.frame_id = "mpu_link"

        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    imu_converter_node = ImuConverterNode()
    rclpy.spin(imu_converter_node)
    
    imu_converter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
