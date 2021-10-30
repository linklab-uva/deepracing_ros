import argparse
from typing import Generator
import skimage
import skimage.io as io
import os
import time
from concurrent import futures
import logging
import argparse
import lmdb
import deepracing.backend
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
from deepracing_ros.controls.pure_puresuit_control_ros import PurePursuitControllerROS as PPC
import deepracing_models.math_utils as mu
import torch
import torch.nn as NN
import torch.utils.data as data_utils
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Header
from nav_msgs.msg import Path
from autoware_auto_msgs.msg import Trajectory, TrajectoryPoint, Complex32
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.clock import Clock, ROSClock
import deepracing_models.nn_models.Models as M
from scipy.spatial.transform import Rotation as Rot
from scipy.spatial.kdtree import KDTree
import cv2, numpy as np
from scipy.spatial import KDTree
from copy import deepcopy
import json
import torch
from deepracing_msgs.srv import SetPurePursuitPath
import deepracing_ros.convert as C
import deepracing.raceline_utils as raceline_utils
import pickle as pkl
import tf2_ros
import deepracing_models.math_utils as mu
import sensor_msgs_py.point_cloud2
import math

class OraclePurePursuitControllerROS(PPC):
    def __init__(self):
        super(OraclePurePursuitControllerROS, self).__init__()
        #Say hello to all of the wonderful people
        self.get_logger().info("Hello Pure Pursuit!")


        
        gpu_param_descriptor = ParameterDescriptor(description="Which gpu to use for computation. Any negative number means use CPU")
        gpu_param : Parameter = self.declare_parameter("gpu", value=-1, descriptor=gpu_param_descriptor)
        self.gpu : int = gpu_param.get_parameter_value().integer_value
        if torch.has_cuda and self.gpu>=0:
            self.device = torch.device("cuda:%d" % self.gpu)
            self.get_logger().info("Running on gpu %d" % (self.gpu,))
        else:
            self.device = torch.device("cpu")
            self.get_logger().info("Running on the cpu" )

        self.raceline = None
        self.dsfinal = None
        self.racelineframe = None
        self.racelinespeeds = None
        self.racelinedists = None
        self.racelinetimes = None
        self.racelinespline : scipy.interpolate.BSpline = None

        print(self.raceline)
        print(self.racelinedists)
        print(self.racelinetimes)
        print(self.racelinespline)

        plot_param : Parameter = self.declare_parameter("plot", value=False)
        self.plot : bool = plot_param.get_parameter_value().bool_value

        dt_param : Parameter = self.declare_parameter("dt", value=2.75)
        self.dt : float = dt_param.get_parameter_value().double_value

        sample_indices_descriptor : ParameterDescriptor = ParameterDescriptor()
        sample_indices_descriptor.name="sample_indices"
        sample_indices_descriptor.read_only=True
        sample_indices_descriptor.description="How many points to sample on the optimal line"
        sample_indices_param : Parameter = self.declare_parameter(sample_indices_descriptor.name, value=100)
        self.sample_indices : int = sample_indices_param.get_parameter_value().integer_value
        
        bezier_order_param : Parameter = self.declare_parameter("bezier_order", value=3)
        self.bezier_order : int = bezier_order_param.get_parameter_value().integer_value

        self.tf2_buffer : tf2_ros.Buffer = tf2_ros.Buffer(node=self)
        self.tf2_listener : tf2_ros.TransformListener = tf2_ros.TransformListener(self.tf2_buffer, self, spin_thread=False)
        
        self.rlsub : Subscription = self.create_subscription(PointCloud2, "racingline", self.racelineCB, 1)

    def racelineCB(self, raceline_pc : PointCloud2):
        numpoints : int = raceline_pc.width 
        rlgenerator : Generator = sensor_msgs_py.point_cloud2.read_points(raceline_pc, field_names=["x","y","z","time","speed"])
        raceline : torch.Tensor = torch.empty(4, numpoints, dtype=torch.float64, device=self.device)
        raceline[3]=1.0
        racelinetimes : torch.Tensor = torch.empty(numpoints, dtype=torch.float64, device=self.device)
        racelinespeeds : torch.Tensor = torch.empty(numpoints, dtype=torch.float64, device=self.device)
        for (i, p) in enumerate(rlgenerator):
            pt : torch.Tensor = torch.as_tensor(p, dtype=torch.float64, device=self.device)
            raceline[0:3,i] = pt[0:3]
            racelinetimes[i] = pt[3]
            racelinespeeds[i] = pt[4]
        racelinespline : scipy.interpolate.BSpline = scipy.interpolate.make_interp_spline(racelinetimes.cpu().numpy().copy(), raceline[0:3].cpu().numpy().copy().transpose(), bc_type="natural")
        self.racelineframe=raceline_pc.header.frame_id
        self.raceline=raceline
        self.racelinetimes=racelinetimes
        self.racelinespeeds=racelinespeeds
        self.racelinespline=racelinespline   

       
        
    def getTrajectory(self):
        if (self.racelineframe is None)  or (self.raceline is None) or (self.racelinespline is None) or (self.racelinetimes is None) or (self.racelinespeeds is None):
            self.get_logger().info("Returning None because raceline not yet received")
            return None, None, None

        transformmsg = self.tf2_buffer.lookup_transform("base_link", self.racelineframe, Time(), timeout=Duration(seconds=1))
        transform = C.transformMsgToTorch(transformmsg.transform, device=self.device, dtype=self.raceline.dtype)

        raceline_base_link = torch.matmul(transform, self.raceline)
        Imin = torch.argmin(torch.norm(raceline_base_link[0:3],p=2,dim=0))
        t0 = self.racelinetimes[Imin]
        tfit = np.linspace(t0, t0+self.dt, num=self.sample_indices)
        rlpiece = torch.as_tensor(self.racelinespline(tfit % self.racelinetimes[-1].item()), dtype=transform.dtype, device=transform.device)
        tfit = torch.as_tensor(tfit, dtype=transform.dtype, device=transform.device)
        dt = tfit[-1]-tfit[0]
        sfit = (tfit-tfit[0])/dt
        _, bcurve = mu.bezierLsqfit(rlpiece.unsqueeze(0), self.bezier_order, t=sfit.unsqueeze(0))
        
        tsamp = torch.linspace(0.0, 1.0, steps=100, dtype=transform.dtype, device=transform.device).unsqueeze(0)
        Msamp = mu.bezierM(tsamp, self.bezier_order)
        _, bcurvderiv = mu.bezierDerivative(bcurve, t=tsamp)
        _, bcurv2ndderiv = mu.bezierDerivative(bcurve, t=tsamp, order=2)


        positionsbcurve : torch.Tensor = torch.matmul(Msamp, bcurve)[0]
        velocitiesbcurve : torch.Tensor = (bcurvderiv[0]/dt)
        accelsbcurve : torch.Tensor = (bcurv2ndderiv[0]/(dt*dt))
        speedsbcurve : torch.Tensor = torch.norm(velocitiesbcurve, p=2, dim=1)
        unit_tangents : torch.Tensor = velocitiesbcurve/speedsbcurve[:,None]
        longitudinal_accels : torch.Tensor = torch.sum(accelsbcurve*unit_tangents, dim=1)

        path_msg : Path = Path(header=Header(frame_id=self.racelineframe, stamp=transformmsg.header.stamp)) 
        trajectory_msg : Trajectory = Trajectory(header=path_msg.header) 
        for i in range(positionsbcurve.shape[0]):

            xvec = unit_tangents[i]

            pose : PoseStamped = PoseStamped(header=path_msg.header)
            pose.pose.position.x=positionsbcurve[i,0].item()
            pose.pose.position.y=positionsbcurve[i,1].item()
            pose.pose.position.z=positionsbcurve[i,2].item()
            path_msg.poses.append(pose)

            trajpoint : TrajectoryPoint = TrajectoryPoint()
            headingangle = torch.atan2(xvec[2], xvec[0]).item()
            trajpoint.heading=Complex32(real=math.cos(headingangle), imag=math.sin(headingangle))
            trajpoint.x=pose.pose.position.x
            trajpoint.y=pose.pose.position.y
            trajpoint.z=pose.pose.position.z
            trajpoint.longitudinal_velocity_mps=speedsbcurve[i].item()
            trajpoint.acceleration_mps2=longitudinal_accels[i].item()
            fracpart, intpart = math.modf((tsamp[0,i]*dt).item())
            trajpoint.time_from_start=Duration(seconds=int(intpart), nanoseconds=int(fracpart*1.0E9)).to_msg()
            trajectory_msg.points.append(trajpoint)



        

        
        return bcurve, path_msg, trajectory_msg
