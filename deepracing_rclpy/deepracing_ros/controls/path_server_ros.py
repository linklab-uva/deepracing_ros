import numpy as np
import os
import time
import logging
import yaml
import torch
import torchvision
import torchvision.transforms as tf
import deepracing.imutils
import scipy
import scipy.interpolate
import deepracing.pose_utils
import deepracing
import threading
import numpy.linalg as la
import scipy.integrate as integrate
import scipy.spatial
import deepracing_models.math_utils as mu
import torch
import torch.nn as NN
import torch.utils.data as data_utils
import deepracing_models.nn_models.Models
from deepracing_msgs.msg import CarControl, TimestampedPacketCarStatusData, TimestampedPacketCarTelemetryData
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import Vector3Stamped, Vector3, PointStamped, Point, PoseStamped, Pose, Quaternion, PoseArray, Twist, TwistStamped
from sensor_msgs.msg import PointCloud2
from rcl_interfaces.msg import ParameterDescriptor
from scipy.spatial.transform import Rotation as Rot
from std_msgs.msg import Float64, Header
import rclpy, rclpy.subscription, rclpy.publisher
from rclpy.node import Node
from rclpy import Parameter
from rclpy.publisher import Publisher
from rclpy.duration import Duration
from rclpy.timer import Timer, Rate
from rclpy.time import Time
from copy import deepcopy
import sensor_msgs
from scipy.spatial.kdtree import KDTree
# from shapely.geometry import Point as ShapelyPoint, MultiPoint
# from shapely.geometry.polygon import Polygon
# from shapely.geometry import LinearRing
import timeit
import deepracing_ros, deepracing_ros.convert
import tf2_ros


class PathServerROS(Node):
    """
    A class used to represent a Local Path Server.
    Attributes
    ----------
    odom_sub : rclpy.subscription.Subscription 
        A subscription to listen for the current odometry of the car in the global coordinate system
    """
    def __init__(self):
        super(PathServerROS,self).__init__('pure_pursuit_control', allow_undeclared_parameters=False, automatically_declare_parameters_from_overrides=False)
          

        base_link_param : Parameter = self.declare_parameter("base_link", value="base_link")
        self.base_link : str = base_link_param.get_parameter_value().string_value
       
        self.current_odom : Odometry = None
        self.tf2_buffer : tf2_ros.Buffer = tf2_ros.Buffer(cache_time = Duration(seconds=5))
        self.tf2_listener : tf2_ros.TransformListener = tf2_ros.TransformListener(self.tf2_buffer, self, spin_thread=False)
        self.odom_sub : rclpy.subscription.Subscription = self.create_subscription(Odometry, 'car_odom', self.odomCallback, 1)

    def odomCallback(self, odom_msg : Odometry):
        self.get_logger().debug("Got a new pose: " + str(odom_msg))
        self.current_odom = odom_msg        

    def getTrajectory(self):
        raise NotImplementedError("Subclasses of PathServerROS must override getTrajectory")
    
    # def getControl(self) -> CarControl:
        
    #     lookahead_positions, v_local_forward, distances_forward_ = self.getTrajectory()
    #     now = self.get_clock().now()
    #     if lookahead_positions is None:
    #         self.get_logger().error("Returning None because lookahead_positions is None")
    #         return self.prev_control, None
    #     if v_local_forward is None:
    #         self.get_logger().error("Returning None because v_local_forward is None")
    #         return self.prev_control, lookahead_positions
    #     if self.velocity_semaphore.acquire(timeout=1.0):
    #         current_velocity = deepcopy(self.current_velocity)
    #         self.velocity_semaphore.release()
    #     else:
    #         self.get_logger().error("Returning None because unable to acquire velocity semaphore")
    #         return self.prev_control, lookahead_positions
    #     if self.current_pose is None:
    #         stamp = self.get_clock().now().to_msg()
    #     else:
    #         stamp = self.current_pose.header.stamp
    #     if self.publish_paths:
    #         pathheader = Header(stamp = stamp, frame_id=self.base_link)
    #         self.path_pub.publish(Path(header=pathheader, poses=[PoseStamped(header=pathheader, pose=Pose(position=Point(x=lookahead_positions[i,0].item(),y=lookahead_positions[i,1].item(),z=lookahead_positions[i,2].item()))) for i in range(lookahead_positions.shape[0])]))
    #     current_velocity_np = np.array([current_velocity.twist.linear.x, current_velocity.twist.linear.y, current_velocity.twist.linear.z])
    #     current_speed = np.linalg.norm(current_velocity_np)
        
    #     if distances_forward_ is None:
    #         distances_forward = torch.norm(lookahead_positions, p=2, dim=1)
    #     else:
    #         distances_forward = distances_forward_


    #     speeds = torch.norm(v_local_forward, p=2, dim=1)
    #     lookahead_distance = max(self.get_parameter("lookahead_gain").get_parameter_value().double_value*current_speed, 2.0)
    #     lookahead_distance_vel = self.get_parameter("velocity_lookahead_gain").get_parameter_value().double_value*current_speed

    #     lookahead_index = torch.argmin(torch.abs(distances_forward-lookahead_distance))
    #     lookahead_index_vel = torch.argmin(torch.abs(distances_forward-lookahead_distance_vel))

    #     lookaheadVector = lookahead_positions[lookahead_index]
    #     D = torch.norm(lookaheadVector, p=2)
    #     lookaheadDirection = lookaheadVector/D
    #     alpha = torch.atan2(lookaheadDirection[1],lookaheadDirection[0])
        
    #     full_lock_right = self.get_parameter("full_lock_right").get_parameter_value().double_value
    #     full_lock_left = self.get_parameter("full_lock_left").get_parameter_value().double_value
    #     left_steer_factor = self.get_parameter("left_steer_factor").get_parameter_value().double_value
    #     right_steer_factor = self.get_parameter("right_steer_factor").get_parameter_value().double_value
    #     max_steer_delta = self.get_parameter("max_steer_delta").get_parameter_value().double_value

    #     physical_angle = np.clip((torch.atan((2 * self.L*torch.sin(alpha)) / D)).item(), self.previous_steering - max_steer_delta, self.previous_steering + max_steer_delta)
    #     physical_angle = np.clip(physical_angle, full_lock_right, full_lock_left)
    #     self.previous_steering = physical_angle
    #     if physical_angle>0:
    #         delta = left_steer_factor*physical_angle
    #     else:
    #         delta = right_steer_factor*physical_angle
    #     # if self.drs_allowed:
    #     #     velsetpoint = 1.0E5
    #     #     self.prev_control = CarControl(header = Header(stamp = stamp, frame_id=self.base_link), steering=delta, throttle=1.0, brake=0.0)
    #     #     return self.prev_control, lookahead_positions
    #     # if self.drs_enabled:
    #     #     velsetpoint = 1.0E5
    #     #     self.prev_control = CarControl(header = Header(stamp = stamp, frame_id=self.base_link), steering=delta, throttle=1.0, brake=0.0)
    #     #     return self.prev_control, lookahead_positions
    #     velsetpoint = speeds[lookahead_index_vel].item()
    #     if current_speed<velsetpoint:
    #         self.prev_control = CarControl(header = Header(stamp = stamp, frame_id=self.base_link), steering=delta, throttle=1.0, brake=0.0)
    #         return self.prev_control, lookahead_positions
    #     else:
    #         self.prev_control = CarControl(header = Header(stamp = stamp, frame_id=self.base_link), steering=delta, throttle=0.0, brake=1.0)
    #         return self.prev_control, lookahead_positions
