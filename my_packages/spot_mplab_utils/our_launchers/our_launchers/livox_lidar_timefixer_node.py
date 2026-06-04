import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy
from sensor_msgs.msg import Imu, PointCloud2

class LLTF(Node):
    def __init__(self):
        super().__init__('livox_lidar_timefixer_node')

        # Declare and get parameters
        self.declare_parameter('Topic_Sub_LidarImu', '/livox/imu/old')
        self.declare_parameter('Topic_Pub_LidarImu', '/livox/imu')
        self.declare_parameter('Topic_Sub_LidarCloud', '/livox/lidar/old')
        self.declare_parameter('Topic_Pub_LidarCloud', '/livox/lidar')

        topic_sub_imu = self.get_parameter('Topic_Sub_LidarImu').get_parameter_value().string_value
        topic_pub_imu = self.get_parameter('Topic_Pub_LidarImu').get_parameter_value().string_value
        topic_sub_cloud = self.get_parameter('Topic_Sub_LidarCloud').get_parameter_value().string_value
        topic_pub_cloud = self.get_parameter('Topic_Pub_LidarCloud').get_parameter_value().string_value

        # QoS settings (equivalent to KeepLast(1))
        qos_profile = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST)

        # IMU Pub/Sub
        self.pub_lidar_imu = self.create_publisher(Imu, topic_pub_imu, qos_profile)
        self.sub_lidar_imu = self.create_subscription(
            Imu, 
            topic_sub_imu, 
            self.callback_lidar_imu, 
            qos_profile
        )

        # Cloud Pub/Sub
        self.pub_lidar_cloud = self.create_publisher(PointCloud2, topic_pub_cloud, qos_profile)
        self.sub_lidar_cloud = self.create_subscription(
            PointCloud2, 
            topic_sub_cloud, 
            self.callback_lidar_cloud, 
            qos_profile
        )

    def callback_lidar_imu(self, msg):
        # Check if there are subscribers to avoid unnecessary processing
        if self.pub_lidar_imu.get_subscription_count() == 0:
            return

        # Update timestamp and publish
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_lidar_imu.publish(msg)

    def callback_lidar_cloud(self, msg):
        if self.pub_lidar_cloud.get_subscription_count() == 0:
            return

        # Update timestamp and publish
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_lidar_cloud.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LLTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()