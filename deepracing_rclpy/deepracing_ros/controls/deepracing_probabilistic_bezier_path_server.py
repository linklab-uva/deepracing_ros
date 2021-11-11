import argparse
from scipy.spatial import transform
import skimage
import skimage.io as io
import os
import time
from concurrent import futures
import logging
import lmdb
import json
import deepracing.backend
from numpy_ringbuffer import RingBuffer as RB
import yaml
import torch
import torchvision
import torchvision.transforms as tf
import torch.nn as NN
import torch.utils.data as data_utils
import deepracing.imutils
import scipy
import scipy.interpolate
from rclpy import duration
from rclpy.subscription import Subscription
import tf2_ros
import rpyutils
import numpy as np
with rpyutils.add_dll_directories_from_env("PATH"):
    import cv_bridge
import deepracing.pose_utils
import deepracing
import threading
import numpy.linalg as la
import scipy.integrate as integrate
import socket
import scipy.spatial
import queue
from deepracing_ros.controls.path_server_ros import PathServerROS
from torch.optim import SGD
import deepracing_models.math_utils as mu
from deepracing_models.nn_models.LossFunctions import BoundaryLoss
import deepracing_ros
import deepracing_ros.convert as C
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image, CompressedImage, PointCloud2
from deepracing_msgs.msg import PathRaw, ImageWithPath, BezierCurve, TrajComparison, TimestampedPacketSessionData, TimestampedPacketCarStatusData, TimestampedPacketCarTelemetryData, CarStatusData, CarTelemetryData
from geometry_msgs.msg import Vector3Stamped, Vector3, PointStamped, Point, PoseStamped, Pose, Quaternion, TransformStamped, Transform, PoseArray
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Header
from autoware_auto_msgs.msg import Trajectory, TrajectoryPoint
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.clock import Clock, ROSClock
import deepracing_models.nn_models.Models as M
import deepracing_models.nn_models.VariationalModels as VM
from scipy.spatial.transform import Rotation as Rot
import torch.nn.functional as F
from scipy.spatial.kdtree import KDTree
from copy import deepcopy
from scipy.interpolate import BSpline, make_interp_spline
import torch.distributions as dist
import rclpy.qos
import builtin_interfaces.msg
from sensor_msgs_py.point_cloud2 import read_points, create_cloud
from typing import List, Union
import math

class BezierPredictionPathServerROS(PathServerROS):
    def __init__(self):
        super(BezierPredictionPathServerROS, self).__init__()
        model_file_param = self.declare_parameter("model_file", value=None)
        if (model_file_param.type_==Parameter.Type.NOT_SET):
            raise ValueError("The parameter \"model_file\" must be set for this rosnode")
        model_file = model_file_param.get_parameter_value().string_value
        print("Using model file : " + str(model_file))
        config_file = os.path.join(os.path.dirname(model_file),"config.yaml")
        if not os.path.isfile(config_file):
            config_file = os.path.join(os.path.dirname(model_file),"model_config.yaml")
        if not os.path.isfile(config_file):
            raise FileNotFoundError("Either config.yaml or model_config.yaml must exist in the same directory as the model file")
        with open(config_file,'r') as f:
            config = yaml.load(f, Loader = yaml.SafeLoader)
        input_channels = config["input_channels"]
        context_length = config["context_length"]
        bezier_order = config.get("bezier_order",None)
        sequence_length = config.get("sequence_length",None)
        use_3dconv = config.get("use_3dconv",True)
        fix_first_point = config["fix_first_point"]
        hidden_dim = config["hidden_dimension"]
        self.rosclock = ROSClock()
        with rpyutils.add_dll_directories_from_env("PATH"):
            self.cvbridge : cv_bridge.CvBridge = cv_bridge.CvBridge()
        self.bufferdtpub = self.create_publisher(Float64, "/buffer_dt", 1)
        self.image_sempahore : threading.Semaphore = threading.Semaphore()
        #self.rosclock._set_ros_time_is_active(True)

        use_compressed_images_param : Parameter = self.declare_parameter("use_compressed_images", value=False)
        use_compressed_images : bool = use_compressed_images_param.get_parameter_value().bool_value


        deltaT_param : Parameter = self.declare_parameter("deltaT", value=float(config.get("lookahead_time", 2.0)))
        self.deltaT : float = deltaT_param.get_parameter_value().double_value

        # x_scale_factor_param : Parameter = self.declare_parameter("x_scale_factor", value=1.0)
        # self.xscale_factor : float = x_scale_factor_param.get_parameter_value().double_value

        # z_offset_param : Parameter = self.declare_parameter("z_offset", value=self.L/2.0)
        # self.z_offset : float = z_offset_param.get_parameter_value().double_value

        velocity_scale_param : Parameter = self.declare_parameter("velocity_scale_factor", value=1.0)
        self.velocity_scale_factor : float = velocity_scale_param.get_parameter_value().double_value
        
        num_sample_points_param : Parameter = self.declare_parameter("num_sample_points", value=60)
        self.num_sample_points : int = num_sample_points_param.get_parameter_value().integer_value

        max_centripetal_acceleration_param : Parameter = self.declare_parameter("max_centripetal_acceleration", value=15.0)
        self.max_centripetal_acceleration : float = max_centripetal_acceleration_param.get_parameter_value().double_value

        max_braking_param : Parameter = self.declare_parameter("max_braking", value=15.0)
        self.max_braking : float = max_braking_param.get_parameter_value().double_value


        use_float : Parameter = self.declare_parameter("use_float", value=True)
       
        gpu_param_descriptor = ParameterDescriptor(description="Which gpu to use for computation. Any negative number means use CPU")
        gpu_param : Parameter = self.declare_parameter("gpu", value=-1, descriptor=gpu_param_descriptor)
        self.gpu : int = gpu_param.get_parameter_value().integer_value
        if self.gpu>=0:
            self.device = torch.device("cuda:%d" % self.gpu)
            self.get_logger().info("Running on gpu %d" % (self.gpu,))
        else:
            self.device = torch.device("cpu")
            self.get_logger().info("Running on the cpu" )
        
        
        self.net : VM.VariationalCurvePredictor = VM.VariationalCurvePredictor(fix_first_point=fix_first_point, bezier_order=bezier_order, input_channels=3, hidden_dim=hidden_dim)
        if use_float.get_parameter_value().bool_value:
            self.dtype = torch.float32
            self.net = self.net.float()
        else:
            self.dtype = torch.float64
            self.net = self.net.double()
        self.zero_ones : torch.Tensor = torch.as_tensor([0.0, 1.0], dtype=self.dtype, device=self.device).unsqueeze(0).expand(bezier_order+1,-1)
        self.get_logger().info('Loading model file: %s' % (model_file) )
        self.net.load_state_dict(torch.load(model_file,map_location=torch.device("cpu")), strict=False)
        self.get_logger().info('Loaded model file: %s' % (model_file) )
        self.get_logger().info('Moving model params to device %s' % (str(self.device),))
        self.net = self.net.to(self.device)
        self.net = self.net.eval()

        self.get_logger().info('Moved model params to device %s' % (str(self.device),))
        self.image_buffer = RB(self.net.context_length,dtype=(float,(3,66,200)))
        self.s_torch = torch.linspace(0.0,1.0,steps=self.num_sample_points, dtype=self.dtype, device=self.device).unsqueeze(0)
        self.image_tensor = torch.zeros((context_length, 3, 66, 200), dtype=self.dtype, device=self.device)

        self.bezierM = mu.bezierM(self.s_torch, self.net.bezier_order)
        self.bezierMderiv = mu.bezierM(self.s_torch, self.net.bezier_order-1)
        self.bezierM2ndderiv = mu.bezierM(self.s_torch, self.net.bezier_order-2)
        self.bezierM.requires_grad = False
        if use_compressed_images:
            self.image_sub = self.create_subscription( CompressedImage, '/f1_game/images/compressed', self.addToBuffer, 1)
        else:
            self.image_sub = self.create_subscription( Image, '/f1_game/images', self.addToBuffer, 1)
        
        
        # self.inner_boundary = None
        # self.outer_boundary = None
        # self.inner_boundary_normals = None
        # self.outer_boundary_normals = None

        # self.boundary_loss = BoundaryLoss(time_reduction="all", batch_reduction="all", relu_type="Leaky", alpha=1.0, beta=1.0).type(self.dtype).to(self.device)
    
        num_particles_param : Parameter = self.declare_parameter("num_particles", value=128)
        self.num_particles = num_particles_param.get_parameter_value().integer_value

        gaussian_filtering_steps_param : Parameter = self.declare_parameter("gaussian_filtering_steps", value=0)
        self.gaussian_filtering_steps = gaussian_filtering_steps_param.get_parameter_value().integer_value

        self.image_timestamp : builtin_interfaces.msg.Time = None

    def addToBuffer(self, img_msg : Union[Image, CompressedImage]):
        if type(img_msg)==CompressedImage:
            imnp = self.cvbridge.compressed_imgmsg_to_cv2(img_msg, desired_encoding="rgb8") 
        elif type(img_msg)==Image:
            imnp = self.cvbridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8") 
        else:
            raise TypeError("img_msg must be either Image or CompressedImage")
        if imnp.shape[0]<=0 or imnp.shape[1]<=0 or (not imnp.shape[2]==3) :
            return
        imtorch : torch.Tensor = tf.functional.to_tensor(imnp)
        if self.image_sempahore.acquire(1.0):
            self.image_tensor[0:-1] =  self.image_tensor[1:].clone()
            self.image_tensor[-1] = imtorch.type_as(self.image_tensor).to(self.image_tensor.device)
            self.image_timestamp = img_msg.header.stamp
            self.image_sempahore.release()

    def getTrajectory(self):
        if self.image_timestamp is None:
            self.get_logger().error("Returning none because no images received yet.")
            return None, None, None
        if self.image_sempahore.acquire(1.0):
            imtorch = self.image_tensor.clone()
            imagestamp : builtin_interfaces.msg.Time = deepcopy(self.image_timestamp)
            self.image_sempahore.release()
        else:
            self.get_logger().error("Unable to acquire image sempahor.")
            return None, None, None
        

        with torch.no_grad():
            bezier_control_points, _, _ = self.net(imtorch.unsqueeze(0))
            bezier_control_points = bezier_control_points.flip(dims=[2])
            bezier_control_points_aug = torch.cat([bezier_control_points[0],  self.zero_ones], dim=1)
            try:
                transform_msg : TransformStamped = self.tf2_buffer.lookup_transform("map", "car", Time.from_msg(imagestamp), Duration(seconds=0, nanoseconds=int(0.5*1E9)))
            except:
                self.get_logger().error("Unable to acquire transform from map to car.")
                return None, None, None
            carpose = C.transformMsgToTorch(transform_msg.transform, dtype=self.dtype, device=self.device)
            bezier_global = torch.matmul(bezier_control_points_aug, carpose[0:3].t()).unsqueeze(0)

            bcurve_msg : BezierCurve = C.toBezierCurveMsg(bezier_global[0], Header(frame_id="map", stamp=imagestamp))
            fracpart, intpart = math.modf(self.deltaT)
            bcurve_msg.delta_t = builtin_interfaces.msg.Duration(sec=int(intpart), nanosec=int(fracpart*1E9))
            return bcurve_msg, None, None

            # positionsbcurve : torch.Tensor = torch.matmul(self.bezierM, bezier_global)[0]

            # _, bcurvderiv = mu.bezierDerivative(bezier_global, M=self.bezierMderiv)
            # velocitiesbcurve : torch.Tensor = (bcurvderiv[0]/self.deltaT)
            # speedsbcurve : torch.Tensor = torch.norm(velocitiesbcurve, p=2, dim=1)
            # unit_tangents : torch.Tensor = velocitiesbcurve/speedsbcurve[:,None]

            # _, bcurv2ndderiv = mu.bezierDerivative(bezier_global, M=self.bezierM2ndderiv, order=2)
            # accelerationsbcurve : torch.Tensor = (bcurv2ndderiv[0]/(self.deltaT*self.deltaT))
            # longitudinal_accels : torch.Tensor = torch.sum(accelerationsbcurve*unit_tangents, dim=1)
            # path_msg : Path = Path(header=bcurve_msg.header) 
            # traj_msg : Trajectory = Trajectory(header=path_msg.header) 
            # up : np.ndarray = np.asarray( [0.0, 0.0, 1.0] )
            # unit_tangents_np : np.ndarray = unit_tangents.cpu().numpy()
            # zmean : float = torch.mean(bezier_global[0,:,2]).item()
            # for i in range(positionsbcurve.shape[0]):
            #     pose : PoseStamped = PoseStamped(header=path_msg.header)
            #     traj_point : TrajectoryPoint = TrajectoryPoint()
            #     pose.pose.position.x=traj_point.x=positionsbcurve[i,0].item()
            #     pose.pose.position.y=traj_point.y=positionsbcurve[i,1].item()
            #     pose.pose.position.z=traj_point.z=zmean

            #     Rmat : np.ndarray = np.eye(3)
            #     Rmat[0:3,0]=unit_tangents_np[i]
            #     Rmat[0:3,1]=np.cross(up, Rmat[0:3,0])
            #     Rmat[0:3,1]=Rmat[0:3,1]/np.linalg.norm(Rmat[0:3,1], ord=2)
            #     Rmat[0:3,2]=np.cross(Rmat[0:3,0], Rmat[0:3,1])
            #     rot : Rot = Rot.from_matrix(Rmat)
            #     quat : np.ndarray = rot.as_quat()
            #     quat[0:2]=0.0
            #     quat = quat/np.linalg.norm(quat,ord=2)
            #     pose.pose.orientation.x=float(quat[0])
            #     pose.pose.orientation.y=float(quat[1])
            #     pose.pose.orientation.z=traj_point.heading.imag=float(quat[2])
            #     pose.pose.orientation.w=traj_point.heading.real=float(quat[3])

            #     fracpart, intpart = math.modf((self.deltaT*self.s_torch[0,i]).item())
            #     traj_point.time_from_start=builtin_interfaces.msg.Duration(sec=int(intpart), nanosec=int(fracpart*1E9))
            #     traj_point.longitudinal_velocity_mps=speedsbcurve[i].item()
            #     traj_point.acceleration_mps2=longitudinal_accels[i].item()

            #     path_msg.poses.append(pose)
            #     traj_msg.points.append(traj_point)
            # final_point : TrajectoryPoint = traj_msg.points[-1]
            # bcurve_msg.delta_t = final_point.time_from_start
            # return bcurve_msg, path_msg, traj_msg

            # self.bc_pub.publish(deepracing_ros.convert.toBezierCurveMsg(bezier_control_points_global, current_pose_msg.header))

            # scale_tril =  torch.zeros(bezier_control_points.shape[1], 2, 2, dtype = bezier_control_points.dtype, device = bezier_control_points.device)

            # scale_tril[0,0,0] = 1.0
            # scale_tril[0,1,1] = 1.0

            # scale_tril[1,0,0] = 1.0
            # scale_tril[1,1,1] = 1.0

            # scale_tril[2:,0,0] = 1.0
            # scale_tril[2:,1,1] = 1.0


            # fpglobal = torch.matmul( transform, torch.cat([bezier_control_points[0,-1], torch.as_tensor([0.0, 1.0], device=self.device, dtype=self.dtype) ]) )[0:3]
            # ib, ibnormal, ob, obnormal= self.getTrackBounds(transform, fpglobal)
        
            # ibpoints = ib.unsqueeze(0)
            # ibnormals = ibnormal.unsqueeze(0)
            # obpoints = ob.unsqueeze(0)
            # obnormals = obnormal.unsqueeze(0)
            
            # for i in range(self.gaussian_filtering_steps):

            #     #scale_tril = torch.sqrt(cvb)
            #     mean_particle = bezier_control_points[0]
            #     curve_distribution : dist.MultivariateNormal = dist.MultivariateNormal(mean_particle, scale_tril=scale_tril, validate_args=False)
            #     self.current_particles = curve_distribution.sample((self.num_particles,))

            #     particle_points = torch.matmul(self.bezierM[0], self.current_particles)
            #     _, v_s = mu.bezierDerivative(self.current_particles, M=self.bezierMderiv[0])
            #     v_t=v_s/self.deltaT
            #     speeds = torch.norm(v_t,dim=2,p=2)
            #     unit_tangents = v_t/speeds[:,:,None]

                
            #     # average_speeds = torch.mean(speeds,dim=1)
            #     # max_average_speed = torch.max(average_speeds)
            #     # speed_scores = average_speeds/max_average_speed
            #     # speed_scores = torch.clip(F.softmax(0.875*average_speeds.double(), dim=0), 1E-24, 1.0)
            #     # speed_scores[speed_scores!=speed_scores] = 0.0
            #     speed_scores = torch.ones_like(speeds[:,0])
            #     # speed_scores=speed_scores/torch.max(speed_scores)
            #    # print(speed_scores)


            #     _, a_s = mu.bezierDerivative(self.current_particles, M=self.bezierM2ndderiv.expand(self.current_particles.shape[0],-1,-1), order=2)
            #     a_t=a_s/(self.deltaT*self.deltaT)

            #     linear_accels = torch.sum(a_t*unit_tangents, dim=2)
            #     linear_accel_vecs = unit_tangents*linear_accels[:,:,None]

                
            #     braking = (-linear_accels)
            #     braking_deltas = torch.relu(braking - self.max_braking)
            #     max_braking_deltas, _ = torch.max(braking_deltas, dim=1)
            #     braking_scores = torch.clip(torch.exp(-1.75*max_braking_deltas.double()), 1E-24, 1.0)
            #    # print(braking_scores)
                

            #     centripetal_accel_vecs = a_t - linear_accel_vecs
            #     centripetal_accels = torch.norm(centripetal_accel_vecs, p=2, dim=2)
            #     centripetal_accels[centripetal_accels!=centripetal_accels] = 0.0
            #     ca_deltas = torch.relu(centripetal_accels - self.max_centripetal_acceleration)
            #     max_ca_deltas, _ = torch.max(ca_deltas, dim=1)
            #     ca_scores = torch.clip(torch.exp(-0.5*max_ca_deltas.double()), 1E-24, 1.0)
            #     # ca_scores = torch.ones_like(speeds[:,0])
            #     #print(ca_scores)

            #     _, ib_distances = self.boundary_loss(particle_points, ibpoints.expand(particle_points.shape[0], -1, -1), ibnormals.expand(particle_points.shape[0], -1, -1))
            #     ib_max_distances, _ = torch.max(ib_distances, dim=1)
            #     ib_max_distances=F.relu(ib_max_distances + 2.0)

            #     _, ob_distances = self.boundary_loss(particle_points, obpoints.expand(particle_points.shape[0], -1, -1), obnormals.expand(particle_points.shape[0], -1, -1))
            #     ob_max_distances, _ = torch.max(ob_distances, dim=1)
            #     ob_max_distances=F.relu(ob_max_distances + 2.0)

            #     all_distances = torch.stack([ib_max_distances, ob_max_distances], dim=0)

            #     overall_max_distances, _ = torch.max(all_distances, dim=0)

            #     boundary_scores = torch.clip( torch.exp(-0.85*overall_max_distances.double()), 1E-24, 1.0)
            #     score_products = ca_scores*speed_scores*boundary_scores*braking_scores

            #     probs = (score_products/torch.sum(score_products))
            #     bezier_control_points = torch.sum(probs[:,None,None]*self.current_particles.double(), dim=0, keepdim=True).type(self.bezierM.dtype)
           # print(bezier_control_points[0].shape)
            #print(zero_ones.shape)
          #  print(bezier_control_points_global)
            # x_samp = torch.matmul(self.bezierM, bezier_control_points)[0]
            # _, vels = mu.bezierDerivative(bezier_control_points, M=self.bezierMderiv)
            # vels=vels[0]/self.deltaT
            # speeds = torch.norm(vels, dim=1)
            # distances_forward = mu.integrate.cumtrapz(speeds.unsqueeze(0), self.s_torch, initial=torch.zeros(1,1,dtype=vels.dtype,device=vels.device))[0]
            # positions = x_samp
            # velocities = vels
            # positions[:,0]+=self.L/2.0
            # print(positions.shape)
        