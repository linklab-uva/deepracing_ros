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
from rclpy.time import Time
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
import deepracing_ros.convert as c
import deepracing.raceline_utils as raceline_utils

class OraclePurePursuitControllerROS(PPC):
    def __init__(self):
        super(OraclePurePursuitControllerROS, self).__init__()
        raceline_file_param : Parameter = self.declare_parameter("raceline_file")
        shift_distance_param : Parameter = self.declare_parameter("shift_distance", value=0.0)
        shift_distance = shift_distance_param.get_parameter_value().double_value
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
        racelinefile_ext = os.path.splitext(os.path.basename(self.raceline_file))[1].lower()
        
        racelinetimes, racelinedists, raceline = raceline_utils.loadRaceline(self.raceline_file)

        self.raceline = raceline.to(self.device)
        self.racelinetimes = racelinetimes.to(torch.device("cpu"))
        self.racelinedists = racelinedists.to(self.device)
        self.racelinespline : scipy.interpolate.BSpline = scipy.interpolate.make_interp_spline(self.racelinetimes.cpu().numpy(),self.raceline[0:3].cpu().numpy().transpose(), bc_type="natural")
        self.racelinesplineder : scipy.interpolate.BSpline = self.racelinespline.derivative()


        plot_param : Parameter = self.declare_parameter("plot", value=False)#,Parameter("plot", value=False))
        self.plot : bool = plot_param.get_parameter_value().bool_value

        dt_param : Parameter = self.declare_parameter("dt", value=2.75)#,Parameter("plot", value=False))
        self.dt : float = dt_param.get_parameter_value().double_value

        # forward_indices_param : Parameter = self.declare_parameter("forward_indices", value=200)#,Parameter("forward_indices", value=120))
        # self.forward_indices : int = forward_indices_param.get_parameter_value().integer_value

        sample_indices_param : Parameter = self.declare_parameter("sample_indices", value=60)#,Parameter("sample_indices", value=120))
        self.sample_indices : int = sample_indices_param.get_parameter_value().integer_value

        # bezier_order_param : Parameter = self.declare_parameter("bezier_order", value=5)#,Parameter("bezier_order", value=7))
        # self.bezier_order : int = bezier_order_param.get_parameter_value().integer_value

        # self.s_torch_lstsq = torch.linspace(0,1,self.forward_indices, dtype=torch.float64, device=self.device).unsqueeze(0)
        # self.bezierMlstsq = mu.bezierM(self.s_torch_lstsq, self.bezier_order)
        
        # self.s_torch_sample = torch.linspace(0,1,self.sample_indices, dtype=torch.float64, device=self.device).unsqueeze(0)
        # self.bezierM = mu.bezierM(self.s_torch_sample, self.bezier_order)
        # # self.bezierMdot = mu.bezierM(self.s_torch_sample, self.bezier_order-1)
        # self.bezierMdotdot = mu.bezierM(self.s_torch_sample, self.bezier_order-2)

        
        additional_translation_param : Parameter = self.declare_parameter("additional_translation", value=[0.0,0.0,-self.L/2])
        additional_translation : np.ndarray = np.array(additional_translation_param.get_parameter_value().double_array_value)
        
        additional_rotation_param : Parameter = self.declare_parameter("additional_rotation", value=[0.0,0.0,0.0,1.0])
        additional_rotation : Rot = Rot.from_quat(np.array(additional_rotation_param.get_parameter_value().double_array_value))

        self.additional_transform = torch.eye(4, device=self.device, dtype=torch.float64)
        self.additional_transform[0:3,0:3] = torch.from_numpy(additional_rotation.as_matrix()).double().to(self.device)
        self.additional_transform[0:3,3] = torch.from_numpy(additional_translation).double().to(self.device)


       # self.setPathService = self.create_service(SetPurePursuitPath, "/pure_pursuit/set_path", self.setPathCB)


        

    # def setPathCB(self, request : SetPurePursuitPath.Request, response : SetPurePursuitPath.Response):
    #     self.get_logger().info("Got a request to update pure pursuit raceline")
    #     response.return_code = SetPurePursuitPath.Response.UNKNOWN_ERROR
    #     try:
    #         pathnp = np.array(list(c.pointCloud2ToNumpy(request.new_path, field_names=["x","y","z"])))
    #     except Exception as e:
    #         response.return_code=SetPurePursuitPath.Response.INVALID_POINT_CLOUD
    #         response.message = str(e.with_traceback())
    #         return response
    #     try:
    #         pathtorch = torch.ones(4, pathnp.shape[0], dtype=torch.float64, device=self.device)
    #         pathtorch[0:3,:] = torch.from_numpy(pathnp.transpose().copy()).double().to(self.device)
    #         pathdiffs = pathnp[1:] - pathnp[0:-1]
    #         pathdiffnorms = np.linalg.norm(pathdiffs, ord=2, axis=1)
    #         pathdists = np.hstack([np.array([0.0]), np.cumsum(pathdiffnorms) ])
    #     except Exception as e:
    #         response.return_code=SetPurePursuitPath.Response.UNKNOWN_ERROR
    #         response.message = str(e.with_traceback())
    #         return response
    #     response.return_code=SetPurePursuitPath.Response.SUCCESS
    #     response.message=""
    #     self.raceline_dists, self.raceline = torch.from_numpy(pathdists).double().to(self.device), pathtorch 
    #     self.get_logger().info("Updated pure pursuit raceline")
    #     return response
        
    def getTrajectory(self):
        if (self.current_pose_mat is None):
            return super().getTrajectory()
        # if self.device == torch.device("cpu"):
        #     current_pose_mat = self.current_pose_mat.clone()
        # else:
    #     current_pose_mat = self.current_pose_mat.to(self.device)
        if self.pose_semaphore.acquire(timeout=1.0):
            current_pose_mat = self.current_pose_mat.to(self.device)
            self.pose_semaphore.release()
        else:
            self.get_logger().error("Unable to acquire semaphore for reading pose data")
            return super().getTrajectory()
        current_pose_mat = torch.matmul(current_pose_mat, self.additional_transform)
        #print("Current pose mat after transform: "+ str(current_pose_mat))
        current_pose_inv = torch.inverse(current_pose_mat)
        raceline_local = torch.matmul(current_pose_inv,self.raceline)
        
        I1 = torch.argmin(torch.norm(raceline_local[0:3],p=2,dim=0)).item()

        t0 = self.racelinetimes[I1].item()
        t1 = t0 + self.dt

        tsamp = np.linspace(t0,t1,self.sample_indices)

        positions_global = torch.from_numpy(self.racelinespline(tsamp%self.racelinetimes[-1].item())).transpose(0,1).to(self.device)
        positions_global_aug = torch.cat([positions_global,torch.ones_like(positions_global[0]).unsqueeze(0)],dim=0)
        velocities_global = torch.from_numpy(self.racelinesplineder(tsamp%self.racelinetimes[-1].item())).transpose(0,1).to(self.device)

        positions = torch.matmul( current_pose_inv, positions_global_aug)
        velocities = torch.matmul( current_pose_inv[0:3,0:3], velocities_global)




        pos = positions[0:3].transpose(0,1)
        vel = velocities.transpose(0,1)
        diffs = pos[1:] - pos[:-1]
     #   print(diffs.shape)
        diffnorms = torch.norm(diffs, p=2, dim=1)
       # print(diffnorms.shape)
        distances_forward = torch.zeros_like(pos[:,0])
        distances_forward[1:] = torch.cumsum(diffnorms,0)


        
        
        return pos, vel, distances_forward



        
        #print(x_samp)
        # return x_samp, v_samp, distances_samp, radii
        