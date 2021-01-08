import cv2
import numpy as np
import argparse
import os
import time
import logging
from numpy_ringbuffer import RingBuffer as RB
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
import socket
import scipy.spatial
import queue
import deepracing_models.math_utils as mu
import torch
import torch.nn as NN
import torch.utils.data as data_utils
import deepracing_models.nn_models.Models
from deepracing_msgs.msg import CarControl
from nav_msgs.msg import Path
from geometry_msgs.msg import Vector3Stamped, Vector3, PointStamped, Point, PoseStamped, Pose, Quaternion, PoseArray, Twist, TwistStamped
from sensor_msgs.msg import PointCloud2
from rcl_interfaces.msg import ParameterDescriptor
from scipy.spatial.transform import Rotation as Rot
from std_msgs.msg import Float64, Header
import rclpy, rclpy.subscription, rclpy.publisher
from rclpy.node import Node
from rclpy import Parameter
from rclpy.publisher import Publisher
from rclpy.timer import Timer, Rate
from copy import deepcopy
import sensor_msgs
from scipy.spatial.kdtree import KDTree
from shapely.geometry import Point as ShapelyPoint, MultiPoint#, Point2d as ShapelyPoint2d
from shapely.geometry.polygon import Polygon
from shapely.geometry import LinearRing
import timeit
import deepracing_ros, deepracing_ros.convert


class PurePursuitControllerROS(Node):
    """
    A class used to represent a Pure Pursuit controller

    ...

    Attributes
    ----------
    pose_sub : rclpy.subscription.Subscription 
        A subscription to listen for the current pose of the car in the global coordinate system,
        converts received messages to a 4X4 pose matrix stored in self.current_pose_mat
    velocity_sub : rclpy.subscription.Subscription
        A subscription to listen for the current velocity of the car in the global coordinate system,
        stores velocity messages in self.current_velocity and the length of the received velocity vector (speed)
        into self.current_velocity
    Methods
    -------
    poseCallback(pose_msg : PoseStamped)
        callback method for pose_sub, do not call this directly. 
    velocityCallback(velocity_msg : TwistStamped)
        callback method for velocity_sub, do not call this directly. 
    getTrajectory():
        This is the core piece of this interface. Users should extend this class and overwrite getTrajectory to
        return a trajectory for the car to follow given the controller's current state
    getTrajectory():
        This is the core piece of this interface. Users should extend this class and overwrite getTrajectory to
        return a trajectory for the car to follow given the controller's current state
    getControl():
        Calls getTrajectory() and returns a deepracing_msgs/CarControl value based on that trajectory using the
        Pure Pursuit Algorithm.  Uses bang/bang control for velocity control with a setpoint velocity set as
        the smaller of the max_speed ros param or the largest speed possible that would not exceed the max_centripetal_acceleration
        ros param at a point on the curve selected with the velocity_lookahead_gain ros param
    """
    def __init__(self):
        super(PurePursuitControllerROS,self).__init__('pure_pursuit_control', allow_undeclared_parameters=False, automatically_declare_parameters_from_overrides=False)
       # self.get_logger().info("Hello Pure Pursuit!")
        self.setpoint_publisher = self.create_publisher(Float64, "vel_setpoint", 1)
        self.velsetpoint = 0.0

        gpu_param_descriptor = ParameterDescriptor(description="Which gpu to use for computation. Any negative number means use CPU")
        gpu_param : Parameter = self.declare_parameter("gpu", value=0, descriptor=gpu_param_descriptor)
        self.gpu : int = gpu_param.get_parameter_value().integer_value
        if self.gpu>=0:
            self.device = torch.device("cuda:%d" % self.gpu)
            self.get_logger().info("Running on gpu %d" % (self.gpu,))
        else:
            self.device = torch.device("cpu")
            self.get_logger().info("Running on the cpu" )

       
        L_param_descriptor = ParameterDescriptor(description="The wheelbase (distance between the axles in meters) of the vehicle being controlled")
        L_param : Parameter = self.declare_parameter("wheelbase", value=3.5, descriptor=L_param_descriptor)
        self.L : float = L_param.get_parameter_value().double_value

        lookahead_gain_param_descriptor = ParameterDescriptor(description="Lookahead gain: linear factor multiplied by current speed to get the lookahead distance for selecting a lookahead point for steering control")
        lookahead_gain_param : Parameter = self.declare_parameter("lookahead_gain",value=0.65, descriptor=lookahead_gain_param_descriptor)
        self.lookahead_gain : float = lookahead_gain_param.get_parameter_value().double_value

        velocity_lookahead_gain_param_descriptor = ParameterDescriptor(description="Velocity Lookahead gain: linear factor multiplied by current speed to get the lookahead distance for selecting a lookahead point for velocity control")
        velocity_lookahead_gain_param : Parameter = self.declare_parameter("velocity_lookahead_gain",value=0.65, descriptor=velocity_lookahead_gain_param_descriptor)
        self.velocity_lookahead_gain : float = velocity_lookahead_gain_param.get_parameter_value().double_value
        
        left_steer_factor_param : Parameter = self.declare_parameter("left_steer_factor",value=3.39814)
        self.left_steer_factor : float = left_steer_factor_param.get_parameter_value().double_value
        
        left_steer_offset_param : Parameter = self.declare_parameter("left_steer_offset",value=0.0)
        self.left_steer_offset : float = left_steer_offset_param.get_parameter_value().double_value
        
        right_steer_factor_param : Parameter = self.declare_parameter("right_steer_factor",value=3.72814)
        self.right_steer_factor : float = right_steer_factor_param.get_parameter_value().double_value
        
        right_steer_offset_param : Parameter = self.declare_parameter("right_steer_offset",value=0.0)
        self.right_steer_offset : float = right_steer_offset_param.get_parameter_value().double_value
        
        use_drs_param : Parameter = self.declare_parameter("use_drs",value=False)
        self.use_drs : bool = use_drs_param.get_parameter_value().bool_value
        
        lateral_dimension_param : Parameter = self.declare_parameter("lateral_dimension", value=0)
        self.lateral_dimension : int = lateral_dimension_param.get_parameter_value().integer_value

        forward_dimension_param : Parameter = self.declare_parameter("forward_dimension", value=2)
        self.forward_dimension : int = forward_dimension_param.get_parameter_value().integer_value
        
        if self.has_parameter("base_link"):
            base_link_param : Parameter = self.get_parameter("base_link")
        else:
            base_link_param : Parameter = self.declare_parameter("base_link", value="rear_axis_middle_ground")

        self.base_link : str = base_link_param.get_parameter_value().string_value

        self.path_pub : Publisher = self.create_publisher(Path, "reference_path", 1)
        
        
        if self.use_drs:
            print("Using DRS")
        else:
            print("Not using DRS")



        self.current_pose : PoseStamped = None
        self.current_velocity : TwistStamped = None
        self.pose_semaphore : threading.Semaphore = threading.Semaphore()
        self.velocity_semaphore : threading.Semaphore = threading.Semaphore()
        self.internal_rate : Rate = self.create_rate(60.0)

        self.initSubscriptions()

    def initSubscriptions(self):
        update_qos = rclpy.qos.QoSProfile(depth=1)
        self.pose_sub : rclpy.subscription.Subscription = self.create_subscription(PoseStamped, 'car_pose', self.poseCallback, update_qos)
        self.velocity_sub : rclpy.subscription.Subscription = self.create_subscription(TwistStamped,'car_velocity',self.velocityCallback, update_qos)

    def poseCallback(self, pose_msg : PoseStamped):
        self.get_logger().debug("Got a new pose: " + str(pose_msg))
        if self.pose_semaphore.acquire(timeout=1.0):
            self.current_pose = pose_msg
            self.pose_semaphore.release()
        else:
            self.get_logger().error("Unable to acquire semaphore to setting the pose data")

    def velocityCallback(self, velocity_msg : TwistStamped):
        self.get_logger().debug("Got a new velocity: " + str(velocity_msg))
        if self.velocity_semaphore.acquire(timeout=1.0):
            self.current_velocity = velocity_msg
            self.velocity_semaphore.release()
        else:
            self.get_logger().error("Unable to acquire semaphore to setting the velocity data")
        

    def getTrajectory(self):
        raise NotImplementedError("Subclasses of PurePursuitControllerROS must override getTrajectory")
    
    def getControl(self) -> CarControl:
        
        lookahead_positions, v_local_forward, distances_forward_ = self.getTrajectory()
        if lookahead_positions is None:
      #      self.get_logger().error("Returning None because lookahead_positions is None")
            return None
        if v_local_forward is None:
            self.get_logger().error("Returning None because v_local_forward is None")
            return None
       
        if self.velocity_semaphore.acquire(timeout=1.0):
            current_velocity = deepcopy(self.current_velocity)
            self.velocity_semaphore.release()
        else:
            self.get_logger().error("Returning None because unable to acquire velocity semaphore")
            return None
        pathheader = Header(stamp = self.get_clock().now(), frame_id=self.base_link)
        self.path_pub.publish(Path(header=pathheader, poses=[PoseStamped(header=pathheader, pose=Pose(position=Point(x=lookahead_positions[i,0].item(),y=lookahead_positions[i,1].item(),z=lookahead_positions[i,2].item()))) for i in range(lookahead_positions.shape[0])]))
        current_velocity_np = np.array([current_velocity.twist.linear.x, current_velocity.twist.linear.y, current_velocity.twist.linear.z])
        current_speed = np.linalg.norm(current_velocity_np)
        
        if distances_forward_ is None:
            distances_forward = la.norm(lookahead_positions, axis=1)
        else:
            distances_forward = distances_forward_


        speeds = torch.norm(v_local_forward, p=2, dim=1)
        lookahead_distance = max(self.lookahead_gain*current_speed, 5.0)
        lookahead_distance_vel = self.velocity_lookahead_gain*current_speed

        lookahead_index = torch.argmin(torch.abs(distances_forward-lookahead_distance))
        lookahead_index_vel = torch.argmin(torch.abs(distances_forward-lookahead_distance_vel))

        lookaheadVector = lookahead_positions[lookahead_index]
        lookaheadVectorVel = lookahead_positions[lookahead_index_vel]

        D = torch.norm(lookaheadVector, p=2)
        lookaheadDirection = lookaheadVector/D
        alpha = torch.atan2(lookaheadDirection[self.lateral_dimension],lookaheadDirection[self.forward_dimension])
        physical_angle = (torch.atan((2 * self.L*torch.sin(alpha)) / D)).item()
        if (physical_angle > 0) :
            delta = self.left_steer_factor*physical_angle + self.left_steer_offset
        else:
            delta = self.right_steer_factor*physical_angle + self.right_steer_offset
        
        self.velsetpoint = speeds[lookahead_index_vel].item()
        self.setpoint_publisher.publish(Float64(data=self.velsetpoint))
        if current_speed<self.velsetpoint:
            return CarControl(steering=delta, throttle=1.0, brake=0.0)
        else:
            return CarControl(steering=delta, throttle=0.0, brake=1.0)
