import argparse
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
from sensor_msgs.msg import Image, CompressedImage
from deepracing_msgs.msg import PathRaw, ImageWithPath
from geometry_msgs.msg import Vector3Stamped, Vector3, PointStamped, Point, PoseStamped, Pose, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Header
import rclpy
from rclpy import Parameter
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

class OraclePurePursuitControllerROS(PPC):
    def __init__(self):
        super(OraclePurePursuitControllerROS, self).__init__()
        raceline_file_param : Parameter = self.declare_parameter("raceline_file")
        if raceline_file_param.type_==Parameter.Type.NOT_SET:
            raise ValueError("\"raceline_file\" parameter not set")
        raceline_file = raceline_file_param.get_parameter_value().string_value
        if  os.path.isabs(raceline_file):
            self.raceline_file = raceline_file
        else:
            envsearchdirs = [s for s in str.split(os.getenv("F1_TRACK_DIRS",""), os.pathsep) if s!=""]
            self.raceline_file = deepracing.searchForFile(raceline_file,envsearchdirs)
        if self.raceline_file is None:
            raise ValueError("\"raceline_file\" parameter must either be an absolute path or in a directory contained in an environment variable F1_TRACK_DIRS")
        racelinefile_base, racelinefile_ext = os.path.splitext(os.path.basename(self.raceline_file))
        
        racelinetimes, racelinedists, raceline = raceline_utils.loadRaceline(self.raceline_file)
        bc_type=([(3, np.zeros(3))], [(3, np.zeros(3))])
        #bc_type="natural"
        #bc_type=None

        self.raceline = raceline.to(self.device)
        self.racelinedists = racelinedists.to(self.device)
        self.racelinetimes = racelinetimes.to(torch.device("cpu"))
        self.racelinespline : scipy.interpolate.BSpline = scipy.interpolate.make_interp_spline(self.racelinetimes.numpy().copy(),self.raceline[0:3].cpu().numpy().copy().transpose(), bc_type=bc_type)
        self.racelinesplineder : scipy.interpolate.BSpline = self.racelinespline.derivative(nu=1)
        self.racelinespline2ndder : scipy.interpolate.BSpline = self.racelinespline.derivative(nu=2)

        plot_param : Parameter = self.declare_parameter("plot", value=False)
        self.plot : bool = plot_param.get_parameter_value().bool_value

        dt_param : Parameter = self.declare_parameter("dt", value=2.75)
        self.dt : float = dt_param.get_parameter_value().double_value

        sample_indices_param : Parameter = self.declare_parameter("sample_indices", value=60)
        self.sample_indices : int = sample_indices_param.get_parameter_value().integer_value

        self.tf2_buffer : tf2_ros.Buffer = tf2_ros.Buffer(node=self)
        self.tf2_listener : tf2_ros.TransformListener = tf2_ros.TransformListener(self.tf2_buffer, self, spin_thread=False)

        base_link_param : Parameter = self.declare_parameter("base_link", value="base_link")
        self.base_link : str = base_link_param.get_parameter_value().string_value

        measurement_link_param : Parameter = self.declare_parameter("measurement_link", value="chassis_centroid")
        self.measurement_link : str = measurement_link_param.get_parameter_value().string_value


       
        
    def getTrajectory(self):
        if (self.current_pose is None):
            return None, None, None

        if self.pose_semaphore.acquire(timeout=1.0):
            current_pose_msg = deepcopy(self.current_pose)
            self.pose_semaphore.release()
        else:
            self.get_logger().error("Unable to acquire semaphore for reading pose data")
            return None, None, None
        
        base_link_to_measurement_link = C.transformMsgToTorch(self.tf2_buffer.lookup_transform(self.base_link, self.measurement_link, Time(), timeout=Duration(seconds=1)).transform, device=self.device, dtype=self.raceline.dtype)
        measurement_link_to_world = torch.inverse(C.poseMsgToTorch(current_pose_msg.pose, device=self.device, dtype=self.raceline.dtype))
        T = torch.matmul(base_link_to_measurement_link, measurement_link_to_world)

        raceline_base_link = torch.matmul(T, self.raceline)
        
        I1 = torch.argmin(torch.norm(raceline_base_link[0:3],p=2,dim=0)).item()

        t0 = self.racelinetimes[I1].item()
        t1 = t0 + self.dt

        tsamp = np.linspace(t0,t1,self.sample_indices)%self.racelinetimes[-1].item()

        positions_global_aug = torch.cat([ torch.as_tensor(self.racelinespline(tsamp), device=T.device, dtype=T.dtype).transpose(0,1),\
                                           torch.ones(tsamp.shape[0], device=T.device, dtype=T.dtype).unsqueeze(0) ], dim=0)       
        velocities_global = torch.as_tensor(self.racelinesplineder(tsamp), device=T.device, dtype=T.dtype).transpose(0,1)

        pos = torch.matmul( T, positions_global_aug)[0:3].transpose(0,1)
        vel = torch.matmul( T[0:3,0:3], velocities_global).transpose(0,1)
        diffs = pos[1:] - pos[:-1]
        diffnorms = torch.norm(diffs, p=2, dim=1)
        distances_forward = torch.zeros_like(pos[:,0])
        distances_forward[1:] = torch.cumsum(diffnorms,0)
        
        return pos, vel, distances_forward



        
        #print(x_samp)
        # return x_samp, v_samp, distances_samp, radii
        