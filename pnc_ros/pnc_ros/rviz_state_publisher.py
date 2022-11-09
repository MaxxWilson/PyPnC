import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)

import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point32

from config.atlas_config import PnCConfig

class RVizStatePublisher():
    def __init__(self, robot_urdf_path, robot_mesh_path):
        rclpy.init()
        self.node = Node("rviz_state_publisher")
        self.joint_state_pub = self.node.create_publisher(JointState, "/joint_states", 10)
        self.transform_pub = self.node.create_publisher(TFMessage, "/tf", 10)
        self.point_cloud_pub = self.node.create_publisher(PointCloud, "/point_cloud", 10)

        # Pinocchio as Dynamics Library
        from pnc.robot_system.pinocchio_robot_system import PinocchioRobotSystem
        self._robot = PinocchioRobotSystem(
            robot_urdf_path,
            robot_mesh_path, False, PnCConfig.PRINT_ROBOT_INFO)

        self._sensor_data = None

    def publish_robot_state(self):
        joint_msg = JointState()
        
        # Set msg time stamp using ros time
        joint_msg.header.stamp = self.node.get_clock().now().to_msg()
        
        # Publish position and velocity of robot joints
        joint_pos = self._sensor_data.get("joint_pos")
        joint_vel = self._sensor_data.get("joint_vel")
        
        for key in joint_pos:
            joint_msg.name.append(key)
            joint_msg.position.append(joint_pos.get(key))
            joint_msg.velocity.append(joint_vel.get(key))
            
        self.joint_state_pub.publish(joint_msg)
        
        # Publish all TF Frames
        tf_msg = TFMessage()
        tf_msg.transforms.append(self.get_tf_env_msg())
        tf_msg.transforms.append(self.get_tf_base_msg())
        tf_msg.transforms.append(self.get_tf_com_msg())
        
        self.transform_pub.publish(tf_msg)

    def get_tf_env_msg(self):
        # Publish world to env base transform
        env_stamped_msg = TransformStamped()
        
        # Set msg time stamp using ros time
        env_stamped_msg.header.stamp = self.node.get_clock().now().to_msg()
        
        env_stamped_msg.header.frame_id = "world"
        env_stamped_msg.child_frame_id = "env_base_link"
        env_stamped_msg.transform.translation.x = 0.0
        env_stamped_msg.transform.translation.y = 0.0
        env_stamped_msg.transform.translation.z = 0.0

        return env_stamped_msg

    def get_tf_base_msg(self):
        # Publish world to pelvis transform
        tf_base_stamped_msg = TransformStamped()
        
        # Set msg time stamp using ros time
        tf_base_stamped_msg.header.stamp = self.node.get_clock().now().to_msg()
        
        tf_base_stamped_msg.header.frame_id = "world"
        tf_base_stamped_msg.child_frame_id = "pelvis"
        tf_base_stamped_msg.transform.translation.x = self._robot._q[0]
        tf_base_stamped_msg.transform.translation.y = self._robot._q[1]
        tf_base_stamped_msg.transform.translation.z = self._robot._q[2]
        tf_base_stamped_msg.transform.rotation.x = self._robot._q[3]
        tf_base_stamped_msg.transform.rotation.y = self._robot._q[4]
        tf_base_stamped_msg.transform.rotation.z = self._robot._q[5]
        tf_base_stamped_msg.transform.rotation.w = self._robot._q[6]

        return tf_base_stamped_msg

    def get_tf_com_msg(self):
        # Publish world to COM transform
        tf_com_stamped_msg = TransformStamped()
        
        # Set msg time stamp using ros time
        tf_com_stamped_msg.header.stamp = self.node.get_clock().now().to_msg()
        
        com_pos = self._robot.get_com_pos()
        
        tf_com_stamped_msg.header.frame_id = "world"
        tf_com_stamped_msg.child_frame_id = "COM"
        tf_com_stamped_msg.transform.translation.x = com_pos[0]
        tf_com_stamped_msg.transform.translation.y = com_pos[1]
        tf_com_stamped_msg.transform.translation.z = com_pos[2]
        tf_com_stamped_msg.transform.rotation.x = 0.0
        tf_com_stamped_msg.transform.rotation.y = 0.0
        tf_com_stamped_msg.transform.rotation.z = 0.0
        tf_com_stamped_msg.transform.rotation.w = 1.0

        return tf_com_stamped_msg

    def update_robot_model(self, sensor_data):
        self._sensor_data = sensor_data
        # Update Encoders
        self._robot.update_system(
            sensor_data["base_com_pos"], sensor_data["base_com_quat"],
            sensor_data["base_com_lin_vel"], sensor_data["base_com_ang_vel"],
            sensor_data["base_joint_pos"], sensor_data["base_joint_quat"],
            sensor_data["base_joint_lin_vel"],
            sensor_data["base_joint_ang_vel"], sensor_data["joint_pos"],
            sensor_data["joint_vel"])

    def publish_point_cloud(self, point_cloud):
        msg = self.get_point_cloud_msg(point_cloud)
        self.point_cloud_pub.publish(msg)

    def get_point_cloud_msg(self, point_cloud):
        msg = PointCloud()
        msg.header.frame_id = "world"
        msg.header.stamp = self.node.get_clock().now().to_msg()

        num_points = np.max(np.shape(point_cloud))
        for i in range(num_points):
            point_msg = Point32()
            point_msg.x = point_cloud[0, i]
            point_msg.y = point_cloud[1, i]
            point_msg.z = point_cloud[2, i]
            msg.points.append(point_msg)
        return msg

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()
        