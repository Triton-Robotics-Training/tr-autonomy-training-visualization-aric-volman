import math

from geometry_msgs.msg import TransformStamped, Transform

from tf_transformations import quaternion_from_euler

import numpy as np

import rclpy
from rclpy.node import Node

import tf_transformations
from vision_msgs.msg import Detection2D, Detection2DArray
from std_msgs.msg import Header

from tf2_ros import TransformBroadcaster

from tr_messages.msg import SimGroundTruth

from tr_messages.msg import DetWithImg

from geometry_msgs.msg import Pose, PoseArray

from geometry_msgs.msg import Quaternion

class FramePublisher(Node):
    def __init__(self):
        # Call constructor and give class a name
        super().__init__('tf2_publisher')

        # Initialize tf2 broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize empty detections
        self.detections_msg = Detection2DArray()
        self.detected_panel = TransformStamped()

        # Initialize empty pose
        self.true_pose_first_robot = Pose()
        self.true_pose_transform = TransformStamped()

        # Initialize empty poses for armor panels
        self.true_pose_second_panels = PoseArray()
        self.panel_0 = TransformStamped()
        self.panel_1 = TransformStamped()
        self.panel_2 = TransformStamped()
        self.panel_3 = TransformStamped()

        # Create subscriptions
        self.subscription_true_pose = self.create_subscription(
            SimGroundTruth,
            f'/simulation/ground_truth',
            self.true_pose_handler,
            1)
        
        self.subscription_detections = self.create_subscription(
            DetWithImg,
            f'/detections',
            self.detections_handler,
            1)

    def true_pose_handler(self, msg):
        self.true_pose_first_robot = msg.primary_robot.camera_pose
        self.true_pose_second_panels = msg.secondary_robot.armor_panel_poses

        # TODO probably dont need these to be member variables 
        if self.true_pose_first_robot:
            self.create_pose_message('map', 'camera_frame',self.true_pose_first_robot, self.true_pose_transform)

        if self.true_pose_second_panels:
            self.create_pose_message('map', 'panel_0', self.true_pose_second_panels[0], self.panel_0)
            self.create_pose_message('map', 'panel_1', self.true_pose_second_panels[1], self.panel_1)
            self.create_pose_message('map', 'panel_2', self.true_pose_second_panels[2], self.panel_2)
            self.create_pose_message('map', 'panel_3', self.true_pose_second_panels[3], self.panel_3)


        # self.get_logger().info(f'Received true pose: "{self.true_pose_first_robot}"') 
        
    def detections_handler(self, msg):
        self.detections_msg = msg.detection_info

        if self.detections_msg:
            detection = self.detections_msg.detections[0]
            if detection.results:
                detected_pose = detection.results[0].pose.pose
                self.create_pose_message('camera_frame', 'detected_panel', detected_pose, self.detected_panel)

        # self.get_logger().info(f'Received detection: "{self.detections_msg}"') 
    
    # Utility function to create message so I don't have to do it manually
    # TODO maybe add type hints for the parameters
    def create_pose_message(self, parent_str, child_str, pose, t):
        
        t.header.frame_id = parent_str
        t.child_frame_id = child_str

        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z

        # TODO nit you can do transform.orientation = pose.orientation directly
        quaternion = pose.orientation

        t.transform.rotation.x = quaternion.x
        t.transform.rotation.y = quaternion.y
        t.transform.rotation.z = quaternion.z
        t.transform.rotation.w = quaternion.w

        # TODO incorrect timestamp
        t.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()