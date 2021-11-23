import argparse
from typing import Generator
from scipy.spatial.transform.rotation import Rotation
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
from deepracing_ros.controls.path_server_ros import PathServerROS
import deepracing_models.math_utils as mu
import torch
import torch.nn as NN
import torch.utils.data as data_utils
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Transform, TransformStamped
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
from deepracing_msgs.msg import BezierCurve
from deepracing_msgs.srv import SetPurePursuitPath
import deepracing_ros
import deepracing_ros.convert as C
import deepracing.raceline_utils as raceline_utils
import pickle as pkl
import tf2_ros
import deepracing_models.math_utils as mu
import sensor_msgs_py.point_cloud2
import math
import builtin_interfaces.msg

class OraclePathServer(PathServerROS):
    def __init__(self):
        super(OraclePathServer, self).__init__()
        #Say hello to all of the wonderful people
        self.get_logger().info("Hello Path Server!")

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


        plot_param : Parameter = self.declare_parameter("plot", value=False)
        self.plot : bool = plot_param.get_parameter_value().bool_value

        dt_param : Parameter = self.declare_parameter("dt", value=2.5)
        self.dt : float = dt_param.get_parameter_value().double_value

        sample_indices_descriptor : ParameterDescriptor = ParameterDescriptor()
        sample_indices_descriptor.name="sample_indices"
        sample_indices_descriptor.read_only=True
        sample_indices_descriptor.description="How many points to sample on the optimal line"
        sample_indices_param : Parameter = self.declare_parameter(sample_indices_descriptor.name, value=100)
        self.sample_indices : int = sample_indices_param.get_parameter_value().integer_value
        
        bezier_order_param : Parameter = self.declare_parameter("bezier_order", value=3)
        self.bezier_order : int = bezier_order_param.get_parameter_value().integer_value

        
        self.rlsub : Subscription = self.create_subscription(PointCloud2, "optimal_raceline", self.racelineCB, 1)
        self.tsamp = torch.linspace(0.0, 1.0, steps=100, dtype=torch.float64, device=self.device).unsqueeze(0)
        self.Msamp = mu.bezierM(self.tsamp, self.bezier_order)
        self.Msampderiv = mu.bezierM(self.tsamp, self.bezier_order-1)
        self.Msamp2ndderiv = mu.bezierM(self.tsamp, self.bezier_order-2)
        
        racelineframe_param : Parameter = self.declare_parameter("raceline_frame", value="map")
        self.racelineframe : str = racelineframe_param.get_parameter_value().string_value

    def racelineCB(self, raceline_pc : PointCloud2):
        numpoints : int = raceline_pc.width 
        rlgenerator : Generator = sensor_msgs_py.point_cloud2.read_points(raceline_pc, field_names=["x","y","z","time","speed"])
        raceline : torch.Tensor = torch.empty(4, numpoints, dtype=self.tsamp.dtype, device=self.device)
        raceline[3]=1.0
        racelinespeeds : torch.Tensor = torch.empty(numpoints, dtype=self.tsamp.dtype, device=self.device)
        racelinetimes : torch.Tensor = torch.zeros_like(racelinespeeds)
        for (i, p) in enumerate(rlgenerator):
            pt : torch.Tensor = torch.as_tensor(p, dtype=self.tsamp.dtype, device=self.device)
            raceline[0:3,i] = pt[0:3]
            racelinetimes[i] = pt[3]
            racelinespeeds[i] = pt[4]
        if raceline_pc.header.frame_id==self.racelineframe:
            racelinemap = raceline
        else:
            transformmsg = self.tf2_buffer.lookup_transform(self.racelineframe, raceline_pc.header.frame_id, Time(), timeout=Duration(seconds=1))
            transform = C.transformMsgToTorch(transformmsg.transform, dtype=raceline.dtype, device=raceline.device)
            racelinemap = torch.matmul(transform, raceline)
        self.raceline=racelinemap
        self.racelinespeeds=racelinespeeds
        self.racelinetimes=racelinetimes
        self.destroy_subscription(self.rlsub)
        self.rlsub=None
        

       
        
    def getTrajectory(self):
        if (self.raceline is None) or (self.racelinetimes is None) or (self.racelinespeeds is None):
            self.get_logger().error("Returning None because raceline not yet received")
            return None, None, None
        if self.current_odom is None:
            self.get_logger().error("Returning None because odometry not yet received")
            return None, None, None
        posemsg = deepcopy(self.current_odom)
        if not (posemsg.header.frame_id==self.racelineframe):
            self.get_logger().error("Returning None because current odometry is in the %f frame, but the raceline is in the %f frame" % (posemsg.header.frame_id,self.racelineframe))
            return None, None, None
        
        map_to_car : torch.Tensor = C.poseMsgToTorch(posemsg.pose.pose, dtype=self.tsamp.dtype, device=self.device)

        if not posemsg.child_frame_id=="base_link":
            car_to_base_link_msg : TransformStamped = self.tf2_buffer.lookup_transform(posemsg.child_frame_id, "base_link", Time.from_msg(posemsg.header.stamp), Duration(seconds=2))
            car_to_base_link : torch.Tensor = C.transformMsgToTorch(car_to_base_link_msg.transform, dtype=self.tsamp.dtype, device=self.device)
            pose_curr : torch.Tensor = torch.matmul(map_to_car, car_to_base_link)
        else:
            pose_curr : torch.Tensor = map_to_car

        raceline_base_link : torch.Tensor = torch.matmul(torch.inverse(pose_curr), self.raceline)
        Imin = torch.argmin(torch.norm(raceline_base_link[0:3], p=2, dim=0))
        t0 = self.racelinetimes[Imin]
        tf = t0+self.dt
        
        if tf<=self.racelinetimes[-1]:
            Imax = torch.argmin(torch.abs(self.racelinetimes - tf))
            tfit = self.racelinetimes[Imin:Imax]
            rlpiece = self.raceline[0:3,Imin:Imax]
        else:
            dsfinal = torch.norm(self.raceline[0:3,0] - self.raceline[0:3,-1], p=2)
            dtfinal = dsfinal/self.racelinespeeds[-1]
            Imax = torch.argmin(torch.abs(self.racelinetimes - (tf - self.racelinetimes[-1])) )
            tfit = torch.cat([self.racelinetimes[Imin:], self.racelinetimes[:Imax]+self.racelinetimes[-1]+dtfinal], dim=0)
            rlpiece = torch.cat([self.raceline[0:3,Imin:], self.raceline[0:3,:Imax]], dim=1)
            
        
        zmean = torch.mean(rlpiece[2]).item()
        tfit = tfit-tfit[0]
        dt = tfit[-1]
        sfit = tfit/dt
        _, bcurve = mu.bezierLsqfit(rlpiece.t().unsqueeze(0), self.bezier_order, t=sfit.unsqueeze(0))

        positionsbcurve : torch.Tensor = torch.matmul(self.Msamp, bcurve)[0]

        _, bcurvderiv = mu.bezierDerivative(bcurve, M=self.Msampderiv)
        velocitiesbcurve : torch.Tensor = (bcurvderiv[0]/dt)
        speedsbcurve : torch.Tensor = torch.norm(velocitiesbcurve, p=2, dim=1)
        unit_tangents : torch.Tensor = velocitiesbcurve/speedsbcurve[:,None]

        _, bcurv2ndderiv = mu.bezierDerivative(bcurve, M=self.Msamp2ndderiv, order=2)
        accelerationsbcurve : torch.Tensor = (bcurv2ndderiv[0]/(dt*dt))
        longitudinal_accels : torch.Tensor = torch.sum(accelerationsbcurve*unit_tangents, dim=1)

        bcurve_msg : BezierCurve = C.toBezierCurveMsg(bcurve[0],posemsg.header)
        path_msg : Path = Path(header=posemsg.header) 
        traj_msg : Trajectory = Trajectory(header=path_msg.header) 
        up : np.ndarray = np.asarray( [0.0, 0.0, 1.0] )
        unit_tangents_np : np.ndarray = unit_tangents.cpu().numpy()
        for i in range(positionsbcurve.shape[0]):
            pose : PoseStamped = PoseStamped(header=path_msg.header)
            traj_point : TrajectoryPoint = TrajectoryPoint()
            pose.pose.position.x=traj_point.x=positionsbcurve[i,0].item()
            pose.pose.position.y=traj_point.y=positionsbcurve[i,1].item()
            pose.pose.position.z=traj_point.z=zmean

            Rmat : np.ndarray = np.eye(3)
            Rmat[0:3,0]=unit_tangents_np[i]
            Rmat[0:3,1]=np.cross(up, Rmat[0:3,0])
            Rmat[0:3,1]=Rmat[0:3,1]/np.linalg.norm(Rmat[0:3,1], ord=2)
            Rmat[0:3,2]=np.cross(Rmat[0:3,0], Rmat[0:3,1])
            rot : Rot = Rot.from_matrix(Rmat)
            quat : np.ndarray = rot.as_quat()
            quat[0:2]=0.0
            quat = quat/np.linalg.norm(quat,ord=2)
            pose.pose.orientation.x=float(quat[0])
            pose.pose.orientation.y=float(quat[1])
            pose.pose.orientation.z=traj_point.heading.imag=float(quat[2])
            pose.pose.orientation.w=traj_point.heading.real=float(quat[3])

            fracpart, intpart = math.modf((dt*self.tsamp[0,i]).item())
            traj_point.time_from_start=builtin_interfaces.msg.Duration(sec=int(intpart), nanosec=int(fracpart*1E9))
            traj_point.longitudinal_velocity_mps=speedsbcurve[i].item()
            traj_point.acceleration_mps2=longitudinal_accels[i].item()

            path_msg.poses.append(pose)
            traj_msg.points.append(traj_point)

        final_point : TrajectoryPoint = traj_msg.points[-1]
        bcurve_msg.delta_t = final_point.time_from_start
        return bcurve_msg, path_msg, traj_msg
