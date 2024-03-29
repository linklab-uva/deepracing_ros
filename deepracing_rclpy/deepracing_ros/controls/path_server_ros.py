from typing import Union
import numpy as np
import os
import time
import logging
import yaml
import torch
import torchvision
import torchvision.transforms as tf
import scipy
import scipy.interpolate
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
from deepracing_msgs.msg import CarControl, TimestampedPacketCarStatusData, TimestampedPacketCarTelemetryData, TimestampedPacketSessionData
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
    def __init__(self, name='path_server_ros'):
        super(PathServerROS,self).__init__(name, allow_undeclared_parameters=False, automatically_declare_parameters_from_overrides=False)
          
        
        carname_param : Parameter = self.declare_parameter("carname", value="")
        self.carname : str = carname_param.get_parameter_value().string_value
        
        self.base_link_id : str = "base_link_%s" %(self.carname,)

        self.player_car_index : int = 0
       
        self.current_odom : Odometry = None
        self.tf2_buffer : tf2_ros.Buffer = tf2_ros.Buffer(cache_time = Duration(seconds=5))
        self.tf2_listener : tf2_ros.TransformListener = tf2_ros.TransformListener(self.tf2_buffer, self, spin_thread=False)
        self.odom_sub : rclpy.subscription.Subscription = self.create_subscription(Odometry, 'odom', self.odomCallback, 1)
        self.session_sub : rclpy.subscription.Subscription = self.create_subscription(TimestampedPacketSessionData, '/session_data', self.sessionCallback, 1)
        self.current_session_data : TimestampedPacketSessionData = None

    def odomCallback(self, odom_msg : Odometry):
        self.get_logger().debug("Got a new pose: " + str(odom_msg))
        self.current_odom = odom_msg        

    def sessionCallback(self, session_msg : TimestampedPacketSessionData):
        self.get_logger().debug("Got a new session packet: " + str(session_msg))
        self.current_session_data = session_msg
        self.player_car_index = session_msg.udp_packet.header.player_car_index       

    def getTrajectory(self):
        raise NotImplementedError("Subclasses of PathServerROS must override getTrajectory")
    