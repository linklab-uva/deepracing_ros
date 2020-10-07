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
        if racelinefile_ext==".json":
            with open(self.raceline_file,"r") as f:
                raceline_dictionary = json.load(f)
            racelinenp = np.column_stack([raceline_dictionary["x"], raceline_dictionary["y"], raceline_dictionary["z"]])
            self.raceline_dists = torch.tensor(raceline_dictionary["dist"]).double().to(self.device)
        elif racelinefile_ext==".csv":
            racelinenp = np.loadtxt(self.raceline_file,dtype=float, skiprows=1,delimiter=",")
            if racelinenp.shape[1]==2:
                racelinenp = np.column_stack([racelinenp, 0.73*np.ones_like(racelinenp[:,0])])
            racelinedistsnp, racelinenp = raceline_utils.shiftRaceline(racelinenp, np.array([0.0,0.0,1.0]), shift_distance)
            # raclinediffs = racelinenp[1:] - racelinenp[:-1]
            # raclinediffnorms = np.linalg.norm(raclinediffs, ord=2, axis=1)
            # racelinedistsnp = np.hstack([np.array([0.0]), np.cumsum(raclinediffnorms)])
            self.raceline_dists = torch.from_numpy(racelinedistsnp).double().to(self.device)
        else:
            raise ValueError("Only .json and .csv extensions are supported")
        self.raceline = torch.stack( [ torch.from_numpy(racelinenp[:,0]),\
                                     torch.from_numpy(racelinenp[:,1]),\
                                     torch.from_numpy(racelinenp[:,2]),\
                                     torch.ones_like(torch.from_numpy(racelinenp[:,0]))], dim=0).double().to(self.device)

        assert(self.raceline_dists.shape[0] == self.raceline.shape[1])


        plot_param : Parameter = self.declare_parameter("plot", value=False)#,Parameter("plot", value=False))
        self.plot : bool = plot_param.get_parameter_value().bool_value

        forward_indices_param : Parameter = self.declare_parameter("forward_indices", value=200)#,Parameter("forward_indices", value=120))
        self.forward_indices : int = forward_indices_param.get_parameter_value().integer_value

        sample_indices_param : Parameter = self.declare_parameter("sample_indices", value=60)#,Parameter("sample_indices", value=120))
        self.sample_indices : int = sample_indices_param.get_parameter_value().integer_value

        bezier_order_param : Parameter = self.declare_parameter("bezier_order", value=5)#,Parameter("bezier_order", value=7))
        self.bezier_order : int = bezier_order_param.get_parameter_value().integer_value

        self.s_torch_lstsq = torch.linspace(0,1,self.forward_indices, dtype=torch.float64, device=self.device).unsqueeze(0)
        self.bezierMlstsq = mu.bezierM(self.s_torch_lstsq, self.bezier_order)
        
        self.s_torch_sample = torch.linspace(0,1,self.sample_indices, dtype=torch.float64, device=self.device).unsqueeze(0)
        self.bezierM = mu.bezierM(self.s_torch_sample, self.bezier_order)
        # self.bezierMdot = mu.bezierM(self.s_torch_sample, self.bezier_order-1)
        self.bezierMdotdot = mu.bezierM(self.s_torch_sample, self.bezier_order-2)

        
        additional_translation_param : Parameter = self.declare_parameter("additional_translation", value=[0.0,0.0,-self.L/2])
        additional_translation : np.ndarray = np.array(additional_translation_param.get_parameter_value().double_array_value)
        
        additional_rotation_param : Parameter = self.declare_parameter("additional_rotation", value=[0.0,0.0,0.0,1.0])
        additional_rotation : Rot = Rot.from_quat(np.array(additional_rotation_param.get_parameter_value().double_array_value))

        self.additional_transform = torch.eye(4, device=self.device, dtype=torch.float64)
        self.additional_transform[0:3,0:3] = torch.from_numpy(additional_rotation.as_matrix()).double().to(self.device)
        self.additional_transform[0:3,3] = torch.from_numpy(additional_translation).double().to(self.device)

        
        forward_dimension_param : Parameter = self.declare_parameter("forward_dimension", value=2)
        self.forward_dimension : int = forward_dimension_param.get_parameter_value().integer_value

        
        lateral_dimension_param : Parameter = self.declare_parameter("lateral_dimension", value=0)
        self.lateral_dimension : int = lateral_dimension_param.get_parameter_value().integer_value

        self.setPathService = self.create_service(SetPurePursuitPath, "/pure_pursuit/set_path", self.setPathCB)


        

    def setPathCB(self, request : SetPurePursuitPath.Request, response : SetPurePursuitPath.Response):
        self.get_logger().info("Got a request to update pure pursuit raceline")
        response.return_code = SetPurePursuitPath.Response.UNKNOWN_ERROR
        try:
            pathnp = np.array(list(c.pointCloud2ToNumpy(request.new_path, field_names=["x","y","z"])))
        except Exception as e:
            response.return_code=SetPurePursuitPath.Response.INVALID_POINT_CLOUD
            response.message = str(e.with_traceback())
            return response
        try:
            pathtorch = torch.ones(4, pathnp.shape[0], dtype=torch.float64, device=self.device)
            pathtorch[0:3,:] = torch.from_numpy(pathnp.transpose().copy()).double().to(self.device)
            pathdiffs = pathnp[1:] - pathnp[0:-1]
            pathdiffnorms = np.linalg.norm(pathdiffs, ord=2, axis=1)
            pathdists = np.hstack([np.array([0.0]), np.cumsum(pathdiffnorms) ])
        except Exception as e:
            response.return_code=SetPurePursuitPath.Response.UNKNOWN_ERROR
            response.message = str(e.with_traceback())
            return response
        response.return_code=SetPurePursuitPath.Response.SUCCESS
        response.message=""
        self.raceline_dists, self.raceline = torch.from_numpy(pathdists).double().to(self.device), pathtorch 
        self.get_logger().info("Updated pure pursuit raceline")
        return response
        
    def getTrajectory(self):
        if (self.current_pose_mat is None):
            return super().getTrajectory()
        # if self.device == torch.device("cpu"):
        #     current_pose_mat = self.current_pose_mat.clone()
        # else:
        #     current_pose_mat = self.current_pose_mat.to(self.device)
        current_pose_mat = torch.matmul( self.current_pose_mat, self.additional_transform )
        #print("Current pose mat after transform: "+ str(current_pose_mat))
        current_pose_inv = torch.inverse(current_pose_mat)
        raceline_local = torch.matmul(current_pose_inv,self.raceline)
        
        I1 = torch.argmin(torch.norm(raceline_local[0:3],p=2,dim=0)).item()

        I2 = I1 + self.forward_indices

        sample_idx = torch.arange(I1, I2, step=1, dtype=torch.int64).to(self.device) % raceline_local.shape[1]

        raceline_points_local = raceline_local[0:3,sample_idx]

        raceline_points_local = raceline_points_local.transpose(0,1).unsqueeze(0)

        _, least_squares_bezier = mu.bezierLsqfit(raceline_points_local, self.bezier_order, M=self.bezierMlstsq)

        least_squares_positions = torch.matmul(self.bezierM, least_squares_bezier)
        least_squares_positions = least_squares_positions[0]
        #distances_forward = torch.norm(least_squares_positions, p=2, dim=1)

        bezierMdot, tsamprdot, least_squares_tangents, least_squares_tangent_norms, distances_forward = mu.bezierArcLength(least_squares_bezier, N=self.sample_indices-1,simpsonintervals=4)
        least_squares_tangents = least_squares_tangents[0]
        least_squares_tangent_norms = least_squares_tangent_norms[0]
        distances_forward = distances_forward[0]

        # _, least_squares_tangents = mu.bezierDerivative(least_squares_bezier, M = bezierMdot, order=1)
        # least_squares_tangents = least_squares_tangents[0]
        # least_squares_tangent_norms = torch.norm(least_squares_tangents, p=2, dim=1)

        _, least_squares_normals = mu.bezierDerivative(least_squares_bezier, M = self.bezierMdotdot, order=2)
        least_squares_normals = least_squares_normals[0]

        radii = torch.pow(least_squares_tangent_norms,3) / torch.norm(torch.cross(least_squares_tangents, least_squares_normals), p=2, dim=1)

        xz_idx = torch.tensor( [self.lateral_dimension,self.forward_dimension] , dtype=torch.int64).to(self.device)

        speeds = self.max_speed*(torch.ones_like(radii)).double().to(self.device)
        centripetal_accelerations = torch.square(speeds)/radii
        max_allowable_speeds = torch.sqrt(self.max_centripetal_acceleration*radii)
        idx = centripetal_accelerations>self.max_centripetal_acceleration
        speeds[idx] = max_allowable_speeds[idx]
        vels = speeds[:,None]*(least_squares_tangents[:,xz_idx]/(least_squares_tangent_norms[:,None]))
        
        return least_squares_positions[:,xz_idx], vels, distances_forward



        
        #print(x_samp)
        # return x_samp, v_samp, distances_samp, radii
        