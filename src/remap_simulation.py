# Bartzialis

import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleControl
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import Odometry
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import Float64
import numpy as np

class Remap_Simulation(Node):
    def __init__(self):
        super().__init__('Remap_System_Simulation')
        # Subscribers
        self.rgb_camera_info_sub = self.create_subscription(CameraInfo, '/carla/ego_vehicle/rgb_front/camera_info', self.rgb_camera_info_callback, 10)
        self.rgb_camera_info_pub = self.create_publisher(CameraInfo, '/zed/zed_node/rgb/camera_info', 10)

        self.depth_camera_info_sub = self.create_subscription(CameraInfo, '/carla/ego_vehicle/depth_front/camera_info', self.depth_camera_info_callback, 10)
        self.depth_camera_info_pub = self.create_publisher(CameraInfo, '/zed/zed_node/depth/camera_info', 10)

        self.rgb_image_sub = self.create_subscription(Image, '/carla/ego_vehicle/rgb_front/image', self.rgb_image_callback, 10)
        self.rgb_image_pub = self.create_publisher(Image, '/zed/zed_node/rgb/image_rect_color', 10)

        self.depth_image_sub = self.create_subscription(Image, '/carla/ego_vehicle/depth_front/image', self.depth_image_callback, 10)
        self.depth_image_pub = self.create_publisher(Image, '/zed/zed_node/depth/depth_registered', 10)

        self.odometry_sub = self.create_subscription(Odometry, '/carla/ego_vehicle/odometry', self.odometry_callback, 10)
        self.odometry_pub = self.create_publisher(Odometry, '/zed/zed_node/odom', 10)

        # Static Tfs
        self.static_tf_ego_vehicle_to_odom = StaticTransformBroadcaster(self)
        self.ego_vehicle_to_odom()
        
        self.static_tf_ego_vehicle = StaticTransformBroadcaster(self)
        self.ego_vehicle_to_zed_camera_link()

        self.static_tf_zed_camera_link = StaticTransformBroadcaster(self)
        self.zed_camera_link_to_zed_camera_center()

        self.static_tf_zed_camera_center = StaticTransformBroadcaster(self)
        self.zed_camera_center_to_zed_left_camera_frame()

        self.static_tf_zed_left_camera_frame = StaticTransformBroadcaster(self)
        self.zed_left_camera_frame_to_zed_left_camera_optical_frame()

        # Vehicle Control Subscribers 
        self.steering_sub = Subscriber(self, Float64, '/steering')
        self.throttle_sub = Subscriber(self, Float64, '/throttle')
        self.brake_sub = Subscriber(self, Float64, '/brake')

        # Time Sunchronizer
        ts = ApproximateTimeSynchronizer([self.steering_sub, self.throttle_sub, self.brake_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.vehicle_control_cmd_callback)

        # Vehicle Control Publisher
        self.control_pub = self.create_publisher(CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd', 10)

    # Callbacks
    def rgb_camera_info_callback(self, msg):
        self.rgb_camera_msg = msg
        self.rgb_camera_msg.header.frame_id = 'zed_left_camera_optical_frame'
        self.rgb_camera_info_pub.publish(self.rgb_camera_msg)

    def depth_camera_info_callback(self, msg):
        self.depth_camera_msg = msg
        self.depth_camera_msg.header.frame_id = 'zed_left_camera_optical_frame'
        self.depth_camera_info_pub.publish(self.depth_camera_msg)

    def rgb_image_callback(self, msg):
        self.rgb_image_msg = msg
        self.rgb_image_msg.header.frame_id = 'zed_left_camera_optical_frame'
        self.rgb_image_pub.publish(self.rgb_image_msg)

    def depth_image_callback(self, msg):
        self.depth_image_msg = msg
        self.depth_image_msg.header.frame_id = 'zed_left_camera_optical_frame'
        self.depth_image_pub.publish(self.depth_image_msg)

    def odometry_callback(self, msg):
        self.odometry_msg = msg
        self.odometry_msg.header.frame_id = 'odom'
        self.odometry_msg.child_frame_id = 'zed_camera_link'
        self.odometry_pub.publish(self.odometry_msg)

    # Tf Def
    def ego_vehicle_to_odom(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'ego_vehicle'
        t.child_frame_id = 'odom'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.static_tf_ego_vehicle_to_odom.sendTransform(t)
        
    def ego_vehicle_to_zed_camera_link(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'ego_vehicle/rgb_front'
        t.child_frame_id = 'zed_camera_link'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.static_tf_ego_vehicle.sendTransform(t)

    def zed_camera_link_to_zed_camera_center(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'zed_camera_link'
        t.child_frame_id = 'zed_camera_center'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.static_tf_zed_camera_link.sendTransform(t)

    def zed_camera_center_to_zed_left_camera_frame(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'zed_camera_center'
        t.child_frame_id = 'zed_left_camera_frame'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.static_tf_zed_camera_center.sendTransform(t)

    def zed_left_camera_frame_to_zed_left_camera_optical_frame(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'zed_left_camera_frame'
        t.child_frame_id = 'zed_left_camera_optical_frame'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.static_tf_zed_left_camera_frame.sendTransform(t)

    # Vehicle Control Callback
    def vehicle_control_cmd_callback(self, steering, throttle, brake):
        control_msg = CarlaEgoVehicleControl()
        control_msg.steer = steering.data
        control_msg.throttle = throttle.data
        control_msg.brake = brake.data
        control_msg.manual_gear_shift = False
        control_msg.gear = 1

        self.control_pub.publish(control_msg)
       




def main(args=None):
    rclpy.init(args=args)

    swap = Remap_Simulation()
    rclpy.spin(swap)

    swap.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
        
