import argparse
from ctypes import ArgumentError
from typing import Generator, Sequence
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
import torch
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
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter
from rclpy.subscription import Subscription
from rclpy.service import Service
from rclpy.publisher import Publisher
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.clock import Clock, ROSClock
import deepracing_models.nn_models.Models as M
from scipy.spatial.transform import Rotation as Rot
from scipy.spatial.kdtree import KDTree
import numpy as np
from scipy.spatial import KDTree
from copy import deepcopy
from typing import List
import json
import torch, torch.distributions
import geometry_msgs.msg
from deepracing_msgs.msg import BezierCurve
import deepracing_ros.convert as C
import deepracing.raceline_utils as raceline_utils
import pickle as pkl
from deepracing_msgs.srv import SetRaceline, GetRaceline
import deepracing_models.math_utils as mu
import math
import builtin_interfaces.msg
import rclpy.exceptions
import ament_index_python
from scipy.spatial.transform import Rotation

class OraclePathServer(PathServerROS):
    def __init__(self):
        super(OraclePathServer, self).__init__()
        #Say hello to all of the wonderful people
        self.get_logger().info("Hello Path Server! I live in namespace: %s" % (self.get_namespace()))

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
        self.racelinetangents = None
        self.racelinenormals = None
        self.dsfinal = None
        self.racelineframe = None
        self.racelinespeeds = None
        self.racelinedists = None
        self.racelinetimes = None


        plot_param : Parameter = self.declare_parameter("plot", value=False)
        self.plot : bool = plot_param.get_parameter_value().bool_value

        dt_param : Parameter = self.declare_parameter("dt", value=2.25)
        self.dt : float = dt_param.get_parameter_value().double_value

        sample_indices_descriptor : ParameterDescriptor = ParameterDescriptor()
        sample_indices_descriptor.name="sample_indices"
        sample_indices_descriptor.read_only=True
        sample_indices_descriptor.description="How many points to sample on the optimal line"
        sample_indices_param : Parameter = self.declare_parameter(sample_indices_descriptor.name, value=100)
        self.sample_indices : int = sample_indices_param.get_parameter_value().integer_value
        
        bezier_order_param : Parameter = self.declare_parameter("bezier_order", value=7)
        self.bezier_order : int = bezier_order_param.get_parameter_value().integer_value

        self.setrlservice : Service = self.create_service(SetRaceline, "set_raceline", self.set_raceline_cb)
        self.getrlservice : Service = self.create_service(GetRaceline, "get_raceline", self.get_raceline_cb)
        self.rlpublisher : Publisher = self.create_publisher(Path, "raceline", 1)
        self.tsamp : torch.Tensor = torch.linspace(0.0, 1.0, steps=30, dtype=torch.float64, device=self.device).unsqueeze(0)
        self.Msamp : torch.Tensor = mu.bezierM(self.tsamp, self.bezier_order)
        self.Msampsquare : torch.Tensor = torch.square(self.Msamp)
        self.Msampderiv : torch.Tensor = mu.bezierM(self.tsamp, self.bezier_order-1)
        self.Msamp2ndderiv : torch.Tensor = mu.bezierM(self.tsamp, self.bezier_order-2)
                
        lateralnoise_param : Parameter = self.declare_parameter("lateralnoise", value=0.0)
        self.lateralnoise : float = lateralnoise_param.get_parameter_value().double_value

        longitudinalnoise_param : Parameter = self.declare_parameter("longitudinalnoise", value=0.0)
        self.longitudinalnoise : float = longitudinalnoise_param.get_parameter_value().double_value
    def rl_as_pathmsg(self) -> Path:
        self.get_logger().info("Getting current raceline from PyTorch tensors")
        normals3d : torch.Tensor = torch.cat([self.racelinenormals, torch.zeros_like(self.racelinenormals[:,0]).unsqueeze(1)], dim=1)
        tangents3d : torch.Tensor = torch.cat([self.racelinetangents, torch.zeros_like(self.racelinetangents[:,0]).unsqueeze(1)], dim=1)
        up3d : torch.Tensor = torch.cross(tangents3d, normals3d, dim=1)
        up3d = up3d/(torch.norm(up3d, p=2, dim=1)[:,None])
        rotmats : torch.Tensor = torch.stack([tangents3d, normals3d, up3d], dim=2)
        rtn : Path = Path()
        rtn.header.stamp = self.get_clock().now().to_msg()
        rtn.header.frame_id = self.racelineframe
        for i in range(rotmats.shape[0]):
            p : PoseStamped = PoseStamped()
            p.header.frame_id = rtn.header.frame_id
            fracpart, intpart  = math.modf(self.racelinetimes[i].item())
            p.header.stamp = builtin_interfaces.msg.Time(sec=int(intpart), nanosec=int(1.0E9*fracpart)) 
            p.pose.position = geometry_msgs.msg.Point(x=self.raceline[i,0].item(), y=self.raceline[i,1].item(), z=self.raceline[i,2].item())
            qnp : np.ndarray = Rot.from_matrix(rotmats[i].cpu().numpy()).as_quat()
            p.pose.orientation = geometry_msgs.msg.Quaternion(x=qnp[0], y=qnp[1], z=qnp[2], w=qnp[3])
            rtn.poses.append(p)
        rtn.poses.append(rtn.poses[0])
        fracpart, intpart = math.modf(self.racelinespline.t[-1])
        rtn.poses[-1].header.stamp = builtin_interfaces.msg.Time(sec=int(intpart), nanosec=int(1.0E9*fracpart)) 
        return rtn
    def get_raceline_cb(self, request : GetRaceline.Request , response : GetRaceline.Response):
        self.get_logger().info("GetRaceline callback")
        if (self.raceline is None) or (self.racelinetangents is None) or (self.racelinenormals is None) or (self.racelinetimes is None) or (self.racelineframe is None):
            response.message="Raceline not yet set"
            response.error_code=GetRaceline.Response.NO_RACELINE_RECEIVED
            return response
        response.current_raceline = self.rl_as_pathmsg()
        response.message="Yay"
        response.error_code=GetRaceline.Response.SUCCESS
        self.rlpublisher.publish(response.current_raceline)
        return response
    def set_raceline_cb(self, request : SetRaceline.Request , response : SetRaceline.Response):
        self.get_logger().info("Processing set raceline request")
        if (request.filename=="") and (len(request.new_raceline.poses)==0):
            response.message="Filename and new raceline cannot both be empty. Exactly 1 of them must be popluated."
            response.error_code=SetRaceline.Response.NEITHER_ARGUMENT_SET
            return response
        if (not (request.filename=="")) and (len(request.new_raceline.poses)>0):
            response.message="Filename and new raceline cannot both be populated. Exactly 1 of them must be popluated."
            response.error_code=SetRaceline.Response.BOTH_ARGUMENTS_SET
            return response
        if not (request.filename==""):
            self.get_logger().info("Loading file: %s" %(request.filename,))
            searchdirs : List[str] = str.split(os.getenv("F1_TRACK_DIRS", ""), os.pathsep)
            try:
                f1_datalogger_dir : str = ament_index_python.get_package_share_directory("f1_datalogger")
                searchdirs.append(os.path.join(f1_datalogger_dir, "tracks", "minimumcurvature"))
                searchdirs.append(os.path.join(f1_datalogger_dir, "tracks"))
            except ament_index_python.PackageNotFoundError:
                pass
            racelinefile : str = deepracing.searchForFile(request.filename, searchdirs)
            if racelinefile is None:
                response.message="File %s not found" % (request.filename,)
                response.error_code=SetRaceline.Response.FILEPATH_NOT_FOUND
                return response
            with open(racelinefile, "r") as f:
                racelinedict : dict = json.load(f)

            self.get_logger().info("Looking up transform")
            try:
                transform_msg : geometry_msgs.msg.TransformStamped = self.tf2_buffer.lookup_transform("map", request.frame_id, time=Time(), timeout=Duration(seconds=5))
            except Exception as e:
                response.message="TF2 lookup error. Underlying exception: %s" % (str(e),)
                response.error_code=SetRaceline.Response.TF_FAILURE
                return response
            self.get_logger().info("Got transform")

            rmat : np.ndarray = Rotation.from_quat([transform_msg.transform.rotation.x, transform_msg.transform.rotation.y, transform_msg.transform.rotation.z, transform_msg.transform.rotation.w]).as_matrix().astype(np.float64)
            transformvec : np.ndarray = np.asarray([transform_msg.transform.translation.x, transform_msg.transform.translation.y, transform_msg.transform.translation.z], dtype=rmat.dtype)
            racelinenp : np.ndarray = np.row_stack([np.asarray(racelinedict["x"], dtype=rmat.dtype), np.asarray(racelinedict["y"], dtype=rmat.dtype), np.asarray(racelinedict["z"], dtype=rmat.dtype)])
            racelinenp = (np.matmul(rmat, racelinenp) + transformvec[:,np.newaxis])
            racelinenp = racelinenp.T
            rsamp : np.ndarray = np.asarray(racelinedict["r"], dtype=racelinenp.dtype)
            racelinespeeds : np.ndarray = np.asarray(racelinedict["speeds"], dtype=racelinenp.dtype)
            velsquares : np.ndarray = np.square(racelinespeeds)
            racelinet : np.ndarray = np.zeros_like(racelinespeeds)
            for i in range(rsamp.shape[0]-1):
                ds : float = rsamp[i+1] - rsamp[i]
                v0 : float = racelinespeeds[i]
                v0square : float = velsquares[i]
                vfsquare : float = velsquares[i+1]
                a0 : float = (vfsquare-v0square)/(2.0*ds)
                if np.abs(a0)>1E-1:
                    deltat : float = (-v0 + np.sqrt(v0square + 2.0*a0*ds))/a0
                else:
                    deltat : float = ds/v0
                racelinet[i+1] = racelinet[i]+deltat
            dsfinal : float = np.linalg.norm(racelinenp[0] - racelinenp[-1], ord=2)
            accelfinal : float = (racelinespeeds[0]**2 - racelinespeeds[-1]**2)/(2.0*dsfinal)
            if np.abs(accelfinal)>1E-1:
                finaldeltat = (-racelinespeeds[-1] + np.sqrt(racelinespeeds[-1]**2 + 2.0*accelfinal*dsfinal))/accelfinal
            else:
                finaldeltat = dsfinal/racelinespeeds[-1]
            finalt = racelinet[-1] + finaldeltat
            self.get_logger().info("finaldeltat: %f" %(finaldeltat,))
            splinet : np.ndarray = np.concatenate([racelinet, np.asarray([finalt])], axis=0)
            splinepts : np.ndarray =  np.concatenate([racelinenp, np.expand_dims(racelinenp[0], axis=0)], axis=0) 
            self.racelinespline : scipy.interpolate.BSpline = scipy.interpolate.make_interp_spline(splinet, splinepts, k=3, bc_type='periodic')
            self.racelinesplineder : scipy.interpolate.BSpline = self.racelinespline.derivative()
            tangent_vectors : np.ndarray = self.racelinesplineder(racelinet)
            tangent_vectors[:,2]=0.0
            tangent_vectors = tangent_vectors/(np.linalg.norm(tangent_vectors, ord=2, axis=1)[:,np.newaxis])
            up : np.ndarray = np.zeros_like(tangent_vectors)
            up[:,2]=1.0
            normal_vectors : np.ndarray = np.cross(up, tangent_vectors)
            normal_vectors[:,2]=0.0
            normal_vectors = normal_vectors/(np.linalg.norm(normal_vectors, ord=2, axis=1)[:,np.newaxis])

            tangent_vectors = tangent_vectors[:,[0,1]]
            normal_vectors = normal_vectors[:,[0,1]]
            
            raceline = torch.ones( (racelinenp.shape[0], 4) , dtype=self.tsamp.dtype, device=self.tsamp.device)
            raceline[:,0:3] = torch.as_tensor(racelinenp, dtype=self.tsamp.dtype, device=self.tsamp.device)
            self.racelineframe = str(transform_msg.header.frame_id)
            self.racelinetimes = torch.as_tensor(racelinet, dtype=self.tsamp.dtype, device=self.tsamp.device)
            self.racelinespeeds = torch.as_tensor(racelinedict["speeds"], dtype=self.tsamp.dtype, device=self.tsamp.device)
            self.racelinetangents = torch.as_tensor(tangent_vectors, dtype=self.tsamp.dtype, device=self.tsamp.device)
            self.racelinenormals = torch.as_tensor(normal_vectors, dtype=self.tsamp.dtype, device=self.tsamp.device)
            self.raceline = raceline
        else:
            try:
                transform_msg : geometry_msgs.msg.TransformStamped = self.tf2_buffer.lookup_transform("map", request.new_raceline.header.frame_id, time=Time(), timeout=Duration(seconds=5))
            except Exception as e:
                response.message="TF2 lookup error. Underlying exception: %s" % (str(e),)
                response.error_code=SetRaceline.Response.TF_FAILURE
                return response
            self.get_logger().info("Got transform")
            transformnp : np.ndarray = np.eye(4, dtype=np.float64)
            transformnp[0:3,3] = np.asarray([transform_msg.transform.translation.x, transform_msg.transform.translation.y, transform_msg.transform.translation.z], dtype=rmat.dtype)
            transformnp[0:3,0:3] = Rotation.from_quat([transform_msg.transform.rotation.x, transform_msg.transform.rotation.y, transform_msg.transform.rotation.z, transform_msg.transform.rotation.w]).as_matrix().astype(np.float64)
            transform : torch.Tensor = torch.from_numpy(transformnp).type_as(self.tsamp).to(self.tsamp.device)
            posemsgs : Sequence[geometry_msgs.msg.PoseStamped] = request.new_raceline.poses
            racelineposes : torch.Tensor = torch.zeros( (len(posemsgs), 4, 4) , dtype=transform.dtype, device=transform.device)
            racelineposes[:,0,0]=racelineposes[:,1,1]=racelineposes[:,2,2]=racelineposes[:,3,3]=1.0
            racelinetimes : torch.Tensor = torch.zeros_like(racelineposes[:,0,0])
            for i in range(racelineposes.shape[0]):
                posein : torch.Tensor = torch.eye(4, dtype=racelineposes.dtype, device=racelineposes.device)
                poseinpositionmsg : geometry_msgs.msg.Point = posemsgs[i].pose.position
                poseinquaternionmsg : geometry_msgs.msg.Quaternion = posemsgs[i].pose.orientation
                posein[0:3,3] = torch.as_tensor([poseinpositionmsg.x, poseinpositionmsg.y, poseinpositionmsg.z], dtype=posein.dtype, device=posein.device)
                posein[0:3,0:3] = torch.as_tensor(Rot.from_quat([poseinquaternionmsg.x, poseinquaternionmsg.y, poseinquaternionmsg.z, poseinquaternionmsg.w]).as_matrix(), dtype=posein.dtype, device=posein.device)
                racelineposes[i] = torch.matmul(transform, posein)
                t_i : Time = Time.from_msg(posemsgs[i].header.stamp)
                racelinetimes[i] = float(t_i.nanoseconds*1E-9)
            spline : scipy.interpolate.BSpline = scipy.interpolate.make_interp_spline(racelinetimes.cpu().numpy(), racelineposes[:,0:3,3].cpu().numpy(), k=3, bc_type="periodic")
            splineder : scipy.interpolate.BSpline = spline.derivative()

            velocities : torch.Tensor = torch.as_tensor(splineder(racelinetimes.cpu().numpy()), dtype=transform.dtype, device=transform.device)
            speeds : torch.Tensor = torch.norm(velocities, p=2, dim=1)
            

            tangent_vectors = posein[:,0:3,0]
            tangent_vectors[:,2]=0.0
            tangent_vectors = tangent_vectors/torch.norm(tangent_vectors, p=2, dim=1)[:,None]

            up : torch.Tensor = torch.zeros_like(tangent_vectors)
            up[:,2]=1.0

            normal_vectors : torch.Tensor = torch.cross(up, tangent_vectors, dim=1)
            normal_vectors[:,2]=0.0
            normal_vectors = normal_vectors/torch.norm(normal_vectors, p=2, dim=1)[:,None]

            tangent_vectors = tangent_vectors[:,[0,1]]
            normal_vectors = normal_vectors[:,[0,1]]
            self.racelineframe = str(transform_msg.header.frame_id)
            self.racelinetimes = racelinetimes[:-1]
            self.racelinespeeds = speeds[:-1]
            self.racelinetangents = tangent_vectors[:-1]
            self.racelinenormals = normal_vectors[:-1]
            self.raceline = racelineposes[:-1,0:3,3]
        self.rlpublisher.publish(self.rl_as_pathmsg())
        response.message="yay"
        response.error_code=SetRaceline.Response.SUCCESS
        return response

    def getTrajectory(self):
        if (self.racelineframe is None) or (self.raceline is None) or (self.racelinetimes is None) or (self.racelinespeeds is None) or (self.racelinetangents is None) or (self.racelinenormals is None):
            self.get_logger().error("Returning None because raceline not yet received")
            return None
        if self.current_odom is None:
            self.get_logger().error("Returning None because odometry not yet received")
            return None
        posemsg = deepcopy(self.current_odom)
        if not (posemsg.header.frame_id==self.racelineframe):
            self.get_logger().error("Returning None because current odometry is in the %f frame, but the raceline is in the %f frame" % (posemsg.header.frame_id,self.racelineframe))
            return None
        map_to_car : torch.Tensor = C.poseMsgToTorch(posemsg.pose.pose, dtype=self.tsamp.dtype, device=self.device)

        if not posemsg.child_frame_id==self.base_link_id:
            car_to_base_link_msg : TransformStamped = self.tf2_buffer.lookup_transform(posemsg.child_frame_id, self.base_link_id, Time.from_msg(posemsg.header.stamp), Duration(seconds=2))
            car_to_base_link : torch.Tensor = C.transformMsgToTorch(car_to_base_link_msg.transform, dtype=self.tsamp.dtype, device=self.device)
            pose_curr : torch.Tensor = torch.matmul(map_to_car, car_to_base_link)
        else:
            pose_curr : torch.Tensor = map_to_car
        pose_curr_inv : torch.Tensor = torch.inverse(pose_curr)
        Imin = torch.argmin(torch.norm(self.raceline[:,0:3] - pose_curr[0:3,3], p=2, dim=1))
        t0 = self.racelinetimes[Imin]
        tf = t0+self.dt
        tvec : np.ndarray = np.linspace(t0, tf, num=300)
        rlpiece : torch.Tensor = torch.from_numpy(self.racelinespline(tvec)).type_as(self.tsamp).to(self.tsamp.device)
        rlpiecebaselink : torch.Tensor = torch.matmul(rlpiece, pose_curr_inv[0:3,0:3].t()) + pose_curr_inv[0:3,3]
        
        # if tf<=self.racelinetimes[-1]:
        #     Imax = torch.argmin(torch.abs(self.racelinetimes - tf))
        #     tfit = self.racelinetimes[Imin:Imax]
        #     rlpiecebaselink = raceline_base_link[Imin:Imax]
        # else:
        #     dsfinal = torch.norm(self.raceline[0,0:3] - self.raceline[-1,0:3], p=2)
        #     dtfinal = dsfinal/self.racelinespeeds[-1]
        #     Imax = torch.argmin(torch.abs(self.racelinetimes - (tf - self.racelinetimes[-1])) )
        #     tfit = torch.cat([self.racelinetimes[Imin:], self.racelinetimes[:Imax]+self.racelinetimes[-1]+dtfinal], dim=0)
        #     rlpiecebaselink = torch.cat([raceline_base_link[Imin:], raceline_base_link[:Imax]], dim=0)
        # rlpiecebaselink[:,2]=0.0
        # rlpiece = torch.matmul(rlpiecebaselink, pose_curr[0:3].T)

        tfit = torch.from_numpy(tvec).type_as(self.tsamp).to(self.tsamp.device)
        tfit = tfit-tfit[0]
        dt = tfit[-1]
        sfit = tfit/dt
        
        _, bcurve = mu.bezierLsqfit(rlpiecebaselink.unsqueeze(0), self.bezier_order, t=sfit.unsqueeze(0))
        bcurve[0] = torch.matmul(bcurve[0], pose_curr[0:3,0:3].t()) + pose_curr[0:3,3]
        bcurve_msg : BezierCurve = C.toBezierCurveMsg(bcurve[0],posemsg.header)
        fracpart, intpart = math.modf(dt.item())
        bcurve_msg.delta_t = builtin_interfaces.msg.Duration(sec=int(intpart), nanosec=int(fracpart*1E9))
        return bcurve_msg

        # positionsbcurve : torch.Tensor = torch.matmul(self.Msamp, bcurve)[0]

        # _, bcurvderiv = mu.bezierDerivative(bcurve, M=self.Msampderiv)
        # velocitiesbcurve : torch.Tensor = (bcurvderiv[0]/dt)
        # speedsbcurve : torch.Tensor = torch.norm(velocitiesbcurve, p=2, dim=1)
        # unit_tangents : torch.Tensor = velocitiesbcurve/speedsbcurve[:,None]
        # up : torch.Tensor = torch.zeros_like(unit_tangents)
        # up[:,2]=1.0
        # unit_normals : torch.Tensor = torch.cross(up, unit_tangents)
        # unit_normals = unit_normals/(torch.norm(unit_normals, p=2, dim=1)[:,None])
        

        # path_msg : Path = Path(header=posemsg.header) 
        # up : np.ndarray = np.asarray( [0.0, 0.0, 1.0] )
        # unit_tangents_np : np.ndarray = unit_tangents.cpu().numpy()
        # unit_normals_np : np.ndarray = unit_normals.cpu().numpy()
        # for i in range(positionsbcurve.shape[0]):
        #     pose : PoseStamped = PoseStamped(header=path_msg.header)
        #     fracpart, intpart = math.modf((dt*self.tsamp[0,i]).item())
        #     pose.header.stamp = builtin_interfaces.msg.Time(sec=int(intpart), nanosec=int(fracpart*1E9))
        #     pose.pose.position.x=positionsbcurve[i,0].item()
        #     pose.pose.position.y=positionsbcurve[i,1].item()
        #     pose.pose.position.z=positionsbcurve[i,2].item()

        #     Rmat : np.ndarray = np.eye(3)
        #     Rmat[0:3,0]=unit_tangents_np[i]
        #     Rmat[0:3,1]=unit_normals_np[i]
        #     Rmat[0:3,2]=np.cross(Rmat[0:3,0], Rmat[0:3,1])
        #     rot : Rot = Rot.from_matrix(Rmat)
        #     quat : np.ndarray = rot.as_quat()
        #     pose.pose.orientation.x=float(quat[0])
        #     pose.pose.orientation.y=float(quat[1])
        #     pose.pose.orientation.z=float(quat[2])
        #     pose.pose.orientation.w=float(quat[3])

        #     path_msg.poses.append(pose)

        # return bcurve_msg, path_msg
