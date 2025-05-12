import argparse
from ctypes import ArgumentError
from typing import Generator, Sequence
from regex import T
import skimage
import skimage.io as io
import os
import time
from concurrent import futures
import logging
import argparse
import sensor_msgs
import sensor_msgs_py.point_cloud2
import torch
import torchvision.transforms as tf
import deepracing.imutils
import scipy
import scipy.interpolate
import deepracing
import threading
import numpy.linalg as la
import scipy.integrate as integrate
import socket
import scipy.spatial
import queue
from deepracing_ros.controls.path_server_ros import PathServerROS
import deepracing_models.math_utils as mu
import deepracing_models.math_utils.bezier as bezier
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
import numpy as np
from scipy.spatial import KDTree
from copy import deepcopy
from typing import List
import json
import torch, torch.distributions
import geometry_msgs.msg
from deepracing_msgs.msg import BezierCurve, CompositeBezierCurve
import deepracing.path_utils
import deepracing_ros.convert as C
import deepracing.raceline_utils as raceline_utils
from deepracing_msgs.srv import SetRaceline, GetRaceline
import deepracing_models.math_utils as mu
import math
import builtin_interfaces.msg
import rclpy.exceptions
import ament_index_python
import scipy.spatial.transform as transform
import sensor_msgs_py


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
        self.racelinekdtree : KDTree = None
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

        segments_descriptor : ParameterDescriptor = ParameterDescriptor()
        segments_descriptor.name="segments"
        segments_descriptor.read_only=True
        segments_descriptor.description="How many segments to use in the CBC fit"
        segments_param : Parameter = self.declare_parameter(segments_descriptor.name, value=4)
        self.segments : int = segments_param.get_parameter_value().integer_value

        
        
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
        # self.bcurve_pub : Publisher = self.create_publisher(BezierCurve, "oraclebeziercurves", 1)
        self.composite_bcurve_pub : Publisher = self.create_publisher(CompositeBezierCurve, "oraclecompositebeziercurves", 1)

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
            qnp : np.ndarray = transform.Rotation.from_matrix(rotmats[i].cpu().numpy()).as_quat()
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
        if (request.filename=="") and (request.new_raceline.point_step==0):
            response.message="Filename and new raceline cannot both be empty. Exactly 1 of them must be popluated."
            response.error_code=SetRaceline.Response.NEITHER_ARGUMENT_SET
            return response
        if (not (request.filename=="")) and (request.new_raceline.point_step>0):
            response.message="Filename and new raceline cannot both be populated. Exactly 1 of them must be popluated."
            response.error_code=SetRaceline.Response.BOTH_ARGUMENTS_SET
            return response
        if not (request.filename==""):
            
            if not os.path.isfile(request.filename):
                response.message="Filename %s doesn't exist." % request.filename
                response.error_code=SetRaceline.Response.FILEPATH_NOT_FOUND
                return response
            
            self.get_logger().info("Loading file: %s" %(request.filename,))

            numpytype, raceline_structured_array, height, width = deepracing.path_utils.loadPCD(request.filename)

            raceline_x : np.ndarray = np.squeeze(raceline_structured_array["x"])
            raceline_y : np.ndarray = np.squeeze(raceline_structured_array["y"])
            raceline_z : np.ndarray = np.squeeze(raceline_structured_array["z"])
            raceline_r : np.ndarray = np.squeeze(raceline_structured_array["arclength"])
            
            splinet = None
            try:
                timein : np.ndarray = np.squeeze(raceline_structured_array["time"])
                splinet = timein.copy()
            except:
                raceline_speed : np.ndarray = np.squeeze(raceline_structured_array["speed"])
                # closed_spline : scipy.interpolate.BSpline = \
                #     scipy.interpolate.make_interp_spline(raceline_timein, np.stack([raceline_x, raceline_y, raceline_z], axis=1), bc_type="periodic", k=2)
                # raceline_velsin = closed_spline(raceline_timein, nu=1)
                # raceline_speed : np.ndarray = np.linalg.norm(raceline_velsin, ord=2.0, axis=1)



            self.get_logger().info("Looking up transform")
            try:
                transform_msg : geometry_msgs.msg.TransformStamped = self.tf2_buffer.lookup_transform("map", request.frame_id, time=Time(), timeout=Duration(seconds=5))
            except Exception as e:
                response.message="TF2 lookup error. Underlying exception: %s" % (str(e),)
                response.error_code=SetRaceline.Response.TF_FAILURE
                return response
            self.get_logger().info("Got transform")

            try:
                rmat : np.ndarray = transform.Rotation.from_quat([transform_msg.transform.rotation.x, transform_msg.transform.rotation.y, transform_msg.transform.rotation.z, transform_msg.transform.rotation.w]).as_matrix().astype(np.float64)
                transformvec : np.ndarray = np.asarray([transform_msg.transform.translation.x, transform_msg.transform.translation.y, transform_msg.transform.translation.z], dtype=rmat.dtype)
                racelinenp : np.ndarray = np.row_stack([raceline_x, raceline_y, raceline_z])
                racelinenp = (np.matmul(rmat, racelinenp) + transformvec[:,np.newaxis])
                racelinenp = racelinenp.T
                if splinet is None:
                    velsquares : np.ndarray = np.square(raceline_speed)
                    racelinet : np.ndarray = np.zeros_like(raceline_speed)
                    for i in range(0, raceline_r.shape[0]-1):
                        ds : float = raceline_r[i+1] - raceline_r[i]
                        v0 : float = raceline_speed[i]
                        vf : float = raceline_speed[i+1]
                        v0square : float = velsquares[i]
                        vfsquare : float = velsquares[i+1]
                        a0 : float = float((vfsquare-v0square)/(2.0*ds))
                        # if a0>=0.0:
                            #positive (or no) acceleration
                        p : np.polynomial.Polynomial = np.polynomial.Polynomial([-ds, v0, 0.5*a0])
                        # else:
                            #negative acceleration (braking)
                            # p : np.polynomial.Polynomial = np.polynomial.Polynomial([-ds, vf, -0.5*a0])
                        allroots = p.roots()
                        realroots = np.real(allroots[np.abs(np.imag(allroots))<1E-6])
                        positiverealroots = realroots[realroots>0.0]
                        deltat : float = float(np.min(positiverealroots))

                        racelinet[i+1] = racelinet[i]+deltat
                    if np.all(racelinenp[0]==racelinenp[-1]):
                        splinet : np.ndarray = racelinet
                        splinepts : np.ndarray = racelinenp
                    else:
                        dsfinal = np.linalg.norm(racelinenp[0] - racelinenp[-1], ord=2)
                        vf = raceline_speed[0] #The "end" of this last segment connecting endpoint to starting point
                        v0 = raceline_speed[-1] #The "start" of this last segment connecting endpoint to starting point
                        afinal : float = float((vf**2 - v0**2)/(2.0*dsfinal))
                        # if afinal>=0.0:
                            #positive (or no) acceleration
                        p : np.polynomial.Polynomial = np.polynomial.Polynomial([-dsfinal, v0, 0.5*afinal])
                        # else:
                            #negative acceleration (braking)
                            # p : np.polynomial.Polynomial = np.polynomial.Polynomial([-dsfinal, vf, -0.5*afinal])
                        allroots = p.roots()
                        realroots = np.real(allroots[np.abs(np.imag(allroots))<1E-6])
                        positiverealroots = realroots[realroots>0.0]
                        finaldeltat : float = float(np.min(positiverealroots))
                        finalt = racelinet[-1] + finaldeltat
                        self.get_logger().info("finaldeltat: %f" %(finaldeltat,))
                        splinet : np.ndarray = np.concatenate([racelinet, np.asarray([finalt])], axis=0)
                racelinet=splinet
                splinepts : np.ndarray = racelinenp # np.concatenate([racelinenp, racelinenp[np.newaxis,0]], axis=0) 
                self.racelinespline : scipy.interpolate.BSpline = scipy.interpolate.make_interp_spline(racelinet, splinepts, k=3, bc_type='periodic')
                self.racelinesplineder : scipy.interpolate.BSpline = self.racelinespline.derivative()
                raceline_speed = np.linalg.norm(self.racelinesplineder(racelinet), ord=2.0, axis=1)
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
                self.racelinespeeds = torch.as_tensor(raceline_speed, dtype=self.tsamp.dtype, device=self.tsamp.device)
                self.racelinetangents = torch.as_tensor(tangent_vectors, dtype=self.tsamp.dtype, device=self.tsamp.device)
                self.racelinenormals = torch.as_tensor(normal_vectors, dtype=self.tsamp.dtype, device=self.tsamp.device)
                self.racelinekdtree = KDTree(raceline[:,0:3].cpu().numpy())
                self.raceline = raceline
            except Exception as e:
                response.message="Unknown error. Underlying exception: %s" % (str(e),)
                response.error_code=SetRaceline.Response.UNKNOWN_ERROR
                return response
        else:
            try:
                transform_msg : geometry_msgs.msg.TransformStamped = self.tf2_buffer.lookup_transform("map", request.new_raceline.header.frame_id, time=Time(), timeout=Duration(seconds=5))
            except Exception as e:
                response.message="TF2 lookup error. Underlying exception: %s" % (str(e.with_traceback(None)),)
                response.error_code=SetRaceline.Response.TF_FAILURE
                return response
            self.get_logger().info("Got transform")
            transform_T : np.ndarray = np.asarray([transform_msg.transform.translation.x, transform_msg.transform.translation.y, transform_msg.transform.translation.z], dtype=float)
            transform_Rot = transform.Rotation.from_quat([transform_msg.transform.rotation.x, transform_msg.transform.rotation.y, transform_msg.transform.rotation.z, transform_msg.transform.rotation.w])
            self.get_logger().info("transform_T: " + str(transform_T))
            self.get_logger().info("transform_Rot: " + str(transform_Rot.as_quat()))
            field_names = {a.name for a in request.new_raceline.fields}
            structured_cloud_in : np.ndarray = sensor_msgs_py.point_cloud2.read_points(request.new_raceline)
            cloud_xyz_in = np.ones([len(structured_cloud_in), 3], dtype=transform_T.dtype)
            cloud_xyz_in[:,0]  = np.squeeze(structured_cloud_in["x"])
            cloud_xyz_in[:,1]  = np.squeeze(structured_cloud_in["y"])
            cloud_xyz_in[:,2]  = np.squeeze(structured_cloud_in["z"])
            cloud_xyz : np.ndarray = transform_Rot.apply(cloud_xyz_in) + transform_T
            cloud_time : np.ndarray = np.squeeze(structured_cloud_in["time"])
            self.racelinespline : scipy.interpolate.BSpline = scipy.interpolate.make_interp_spline(cloud_time, cloud_xyz, k=3, bc_type='periodic')
            self.racelinesplineder : scipy.interpolate.BSpline = self.racelinespline.derivative()
            velocity_vectors = self.racelinesplineder(cloud_time)
            tangent_vectors = velocity_vectors/np.linalg.norm(velocity_vectors, ord=2.0, axis=-1, keepdims=True)
            up = np.zeros_like(tangent_vectors)
            up[:,-1] = 1.0
            normal_vectors = np.cross(up, tangent_vectors)
            normal_vectors /= np.linalg.norm(normal_vectors, ord=2.0, axis=-1, keepdims=True)
            if "speed" in field_names:
                self.get_logger().info("Using speeds contained in the point cloud")
                self.racelinespeeds = np.squeeze(structured_cloud_in["speed"])
            else:
                self.get_logger().info("Computing speed by spline interpolation")
                cloud_arclength : np.ndarray = np.squeeze(structured_cloud_in["arclength"])
                arclengthspline = scipy.interpolate.make_interp_spline(cloud_time, cloud_arclength, k=2)
                self.racelinespeeds = arclengthspline(cloud_time, nu=1)
            self.racelineframe = str(transform_msg.header.frame_id)
            self.racelinetangents = tangent_vectors
            self.racelinenormals = normal_vectors

            # dt = 0.01
            # Nsamp = int(round(cloud_time[-1]/dt))
            # self.racelinetimes = np.linspace(0.0, cloud_time[-1], num=Nsamp)
            self.racelinetimes = torch.as_tensor(cloud_time, dtype=torch.float64)
            self.raceline = self.racelinespline(self.racelinetimes.cpu().numpy())
            self.racelinekdtree = KDTree(self.raceline)
        # self.rlpublisher.publish(self.rl_as_pathmsg())
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
            
        _, Imin = self.racelinekdtree.query(pose_curr[0:3,3].cpu().numpy())
        t0 = self.racelinetimes[Imin].item()
        tf = t0+self.dt
        tvec : np.ndarray = np.linspace(t0, tf, num=150)
        rlpiece : torch.Tensor = torch.from_numpy(self.racelinespline(tvec)).type_as(self.tsamp).to(self.tsamp.device)
        rlpiece_local = rlpiece
        # poseinv_T = -(pose_curr[0:3,0:3].T @ pose_curr[0:3,[3,]]).squeeze(-1)
        # rlpiece_local = (rlpiece @ pose_curr[0:3,0:3]) + poseinv_T

        tfit = (torch.as_tensor(tvec).type_as(rlpiece_local) - t0)
        P0 = (pose_curr[0:3,3])
        # P0 = torch.zeros_like(rlpiece_local[0])
        V0 = torch.as_tensor(self.racelinespline(tvec[0], nu=1))[None].type_as(tfit)# @ pose_curr[0:3,0:3]
        # Vf = torch.as_tensor(self.racelinespline(tvec[-1], nu=1))[None].type_as(tfit) @ pose_curr[0:3,0:3]
        # dYdT_0=V0, 
        (controlpoints_fit,), (tswitch,)  = mu.compositeBezierFit(tfit[None], rlpiece_local[None], self.segments, Y_0=P0[None], constraint_level=2)
        delta_t = (tswitch[1:] - tswitch[:-1])#*0.975

        cbc_msg = C.toCompositeBezierCurveMsg(delta_t, controlpoints_fit, header=posemsg.header)
        # cbc_msg.header.frame_id = self.base_link_id
        # cbc_msg.header.frame_id = self.base_link_id
        # cbc_msg.header.stamp = posemsg.header.stamp
        self.composite_bcurve_pub.publish(cbc_msg)

