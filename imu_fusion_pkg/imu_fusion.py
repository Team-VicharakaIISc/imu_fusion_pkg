import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu
from mavros_msgs.msg import GPSRAW
from sensor_msgs.msg import NavSatFix


class IMUFusionNode(Node):
    def __init__(self):
        super().__init__('imu_fusion_node')

        # Define a QoS profile compatible with RealSense topics
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Ensures reliable communication
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )


        # Subscribers with the corrected QoS
        self.gyro_sub = self.create_subscription(Imu, '/camera/camera/gyro/sample', self.gyro_callback, qos_profile)
        self.accel_sub = self.create_subscription(Imu, '/camera/camera/accel/sample', self.accel_callback, qos_profile)
        self.px4_imu_sub = self.create_subscription(Imu, '/mavros/imu/data', self.px4_imu_callback, qos_profile)
        self.px4_gps_sub = self.create_subscription(GPSRAW, '/mavros/gpsstatus/gps1/raw', self.px4_gps_callback, qos_profile)

        # Publisher
        self.depth_imu_pub = self.create_publisher(Imu, '/depth/imu/data', qos_profile)
        self.px4_imu_pub = self.create_publisher(Imu, '/px4/imu/data', qos_profile)
        self.px4_gps_pub = self.create_publisher(NavSatFix, '/px4/gps/fix', qos_profile)

        # Store latest values
        self.latest_gyro = None
        self.latest_accel = None
        self.latest_px4_imu_data= None
        self.latest_px4_gps_data= None
        self.px4_data_calibrated= False
        self.depth_data_calibrated= False
        self.px4_ax_offset= 0
        self.px4_ay_offset= 0
        self.px4_az_offset= 0
        self.px4_gx_offset= 0
        self.px4_gy_offset= 0
        self.px4_gz_offset= 0
        self.depth_ax_offset= 0
        self.depth_ay_offset= 0
        self.depth_az_offset= 0
        self.depth_gx_offset= 0
        self.depth_gy_offset= 0
        self.depth_gz_offset= 0
        self.gravitional = 9.81


    def gyro_callback(self, msg):
        self.latest_gyro = msg
        self.publish_fused_depth_imu()

    def accel_callback(self, msg):
        self.latest_accel = msg
        self.publish_fused_depth_imu()
    
    def px4_imu_callback(self, msg):
        self.latest_px4_imu_data = msg
        self.publish_fused_px4_imu()
    
    def px4_gps_callback(self, msg):
        self.latest_px4_gps_data = msg
        self.publish_px4_gps()

    def publish_fused_depth_imu(self):
        if self.latest_gyro is None or self.latest_accel is None:
            return  # Wait for both messages to be received
        
        if (self.depth_data_calibrated==False):
            self.depth_ay_offset = self.latest_accel.linear_acceleration.y - self.gravitional
            self.depth_ax_offset= self.latest_accel.linear_acceleration.x
            self.depth_az_offset = self.latest_accel.linear_acceleration.z

            self.depth_gy_offset  = self.latest_gyro.angular_velocity.y
            self.depth_gx_offset  = self.latest_gyro.angular_velocity.x
            self.depth_gz_offset  = self.latest_gyro.angular_velocity.z

            self.depth_data_calibrated = True
            
        fused_msg = Imu()
        fused_msg.header.stamp = self.get_clock().now().to_msg()
        fused_msg.header.frame_id = "camera_link"

        # Swapping axes to match correct IMU frame
        fused_msg.linear_acceleration.y = self.latest_accel.linear_acceleration.x 
        fused_msg.linear_acceleration.x = self.latest_accel.linear_acceleration.z 
        fused_msg.linear_acceleration.z = (-1) * (self.latest_accel.linear_acceleration.y ) 

        fused_msg.angular_velocity.y = self.latest_gyro.angular_velocity.x - self.depth_gx_offset 
        fused_msg.angular_velocity.x = self.latest_gyro.angular_velocity.z - self.depth_gz_offset 
        fused_msg.angular_velocity.z = (-1) * ( self.latest_gyro.angular_velocity.y - self.depth_gy_offset )

        fused_msg.angular_velocity_covariance = self.latest_gyro.angular_velocity_covariance
        fused_msg.linear_acceleration_covariance = self.latest_gyro.linear_acceleration_covariance

        # (Optional) Orientation can be left empty or set to identity (0,0,0,1)
        fused_msg.orientation.w = 1.0  # Identity quaternion

        self.depth_imu_pub.publish(fused_msg)
        self.get_logger().info("Published fused depth IMU data")

    def publish_fused_px4_imu(self):
        if self.latest_px4_imu_data is None :
            return  # Wait for both messages to be received
        
        fused_msg = Imu()
        fused_msg.header.stamp = self.get_clock().now().to_msg()
        fused_msg.header.frame_id = "px4_link"

        if (self.px4_data_calibrated==False):
            self.px4_ay_offset = self.latest_px4_imu_data.linear_acceleration.y
            self.px4_ax_offset= self.latest_px4_imu_data.linear_acceleration.x
            self.px4_az_offset = self.latest_px4_imu_data.linear_acceleration.z - self.gravitional

            self.px4_gy_offset  = self.latest_px4_imu_data.angular_velocity.x
            self.px4_gx_offset  = self.latest_px4_imu_data.angular_velocity.z
            self.px4_gz_offset  = self.latest_px4_imu_data.angular_velocity.y

            self.px4_data_calibrated = True

        # Swapping axes to match correct IMU frame
        fused_msg.linear_acceleration.y = self.latest_px4_imu_data.linear_acceleration.y - self.px4_ay_offset
        fused_msg.linear_acceleration.x = self.latest_px4_imu_data.linear_acceleration.x - self.px4_ax_offset
        fused_msg.linear_acceleration.z =  self.latest_px4_imu_data.linear_acceleration.z - self.px4_az_offset

        fused_msg.angular_velocity.y = self.latest_px4_imu_data.angular_velocity.y - self.px4_gy_offset
        fused_msg.angular_velocity.x = self.latest_px4_imu_data.angular_velocity.x - self.px4_gx_offset
        fused_msg.angular_velocity.z =  self.latest_px4_imu_data.angular_velocity.z - self.px4_gz_offset

        # (Optional) Orientation can be left empty or set to identity (0,0,0,1)
        fused_msg.orientation = self.latest_px4_imu_data.orientation
        fused_msg.orientation_covariance = self.latest_px4_imu_data.orientation_covariance
        fused_msg.angular_velocity_covariance = self.latest_px4_imu_data.angular_velocity_covariance
        fused_msg.linear_acceleration_covariance = self.latest_px4_imu_data.linear_acceleration_covariance 
 

        self.px4_imu_pub.publish(fused_msg)
        self.get_logger().info("Published fused px4 IMU data")

    def publish_px4_gps(self):
        if self.latest_px4_gps_data is None :
            return  # Wait for both messages to be received
        
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "px4_link"
        
        # Converting from MAVROS GPSRAW format to NavSatFix
        gps_msg.latitude = self.latest_px4_gps_data.lat * 1e-7  # Convert from int32 to degrees
        gps_msg.longitude = self.latest_px4_gps_data.lon * 1e-7  # Convert from int32 to degrees
        gps_msg.altitude = self.latest_px4_gps_data.alt * 1e-3  # Convert from mm to meters
        
        # Assuming horizontal and vertical accuracy values
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        
        self.px4_gps_pub.publish(gps_msg)
        self.get_logger().info("Published PX4 GPS data")

        


def main(args=None):
    rclpy.init(args=args)
    node = IMUFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
