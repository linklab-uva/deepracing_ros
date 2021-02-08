import argparse
import skimage
import skimage.io as io
import os
import time
from concurrent import futures
import logging
import lmdb
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
import rpyutils
import tf2_ros
with rpyutils.add_dll_directories_from_env("PATH"):
    import py_f1_interface
    import cv_bridge
import deepracing.pose_utils
import deepracing
import threading
import numpy.linalg as la
import scipy.integrate as integrate
import socket
import scipy.spatial
import queue
from deepracing_ros.controls.pure_puresuit_control_ros import PurePursuitControllerROS as PPC
from torch.optim import SGD
import deepracing_models.math_utils as mu
from deepracing_models.nn_models.LossFunctions import BoundaryLoss
import deepracing_ros
import deepracing_ros.convert
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image, CompressedImage, PointCloud2
from deepracing_msgs.msg import PathRaw, ImageWithPath, BezierCurve as BCMessage, TrajComparison, TimestampedPacketCarStatusData, TimestampedPacketCarTelemetryData, CarStatusData, CarTelemetryData
from geometry_msgs.msg import Vector3Stamped, Vector3, PointStamped, Point, PoseStamped, Pose, Quaternion, TransformStamped, Transform
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Header
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
import cv2, numpy as np
import timeit
import array
import torch.nn.functional as F
from scipy.spatial.kdtree import KDTree
from copy import deepcopy
from scipy.interpolate import BSpline, make_interp_spline
import torch.distributions as dist
import rclpy.qos
from sensor_msgs_py.point_cloud2 import read_points, create_cloud

from typing import List


class ProbabilisticBezierPurePursuitControllerROS(PPC):
    def __init__(self):
        super(ProbabilisticBezierPurePursuitControllerROS, self).__init__()
        self.comparison_publisher : Publisher = self.create_publisher(TrajComparison, "/trajectory_comparisons", 1)
        self.ib_local_publisher : Publisher = self.create_publisher(Path, "/inner_boundary_local", 1)
        self.ob_local_publisher : Publisher = self.create_publisher(Path, "/outer_boundary_local", 1)
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
        #self.rosclock._set_ros_time_is_active(True)

        print(config)
        if(self.publish_paths):
            print("Publishing predicted bezier curves")
        else:
            print("Not publishing predicted bezier curves")

        use_compressed_images_param : Parameter = self.declare_parameter("use_compressed_images", value=False)


        deltaT_param : Parameter = self.declare_parameter("deltaT", value=1.44)
        self.deltaT : float = deltaT_param.get_parameter_value().double_value

        x_scale_factor_param : Parameter = self.declare_parameter("x_scale_factor", value=1.0)
        self.xscale_factor : float = x_scale_factor_param.get_parameter_value().double_value

        z_offset_param : Parameter = self.declare_parameter("z_offset", value=self.L/2.0)
        self.z_offset : float = z_offset_param.get_parameter_value().double_value

        velocity_scale_param : Parameter = self.declare_parameter("velocity_scale_factor", value=1.0)
        self.velocity_scale_factor : float = velocity_scale_param.get_parameter_value().double_value
        
        num_sample_points_param : Parameter = self.declare_parameter("num_sample_points", value=60)
        self.num_sample_points : int = num_sample_points_param.get_parameter_value().integer_value

        max_centripetal_acceleration_param : Parameter = self.declare_parameter("max_centripetal_acceleration", value=15.0)
        self.max_centripetal_acceleration : float = max_centripetal_acceleration_param.get_parameter_value().double_value

        
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
        self.get_logger().info('Loading model file: %s' % (model_file) )
        self.net.load_state_dict(torch.load(model_file,map_location=torch.device("cpu")), strict=False)
        self.get_logger().info('Loaded model file: %s' % (model_file) )
        self.get_logger().info('Moving model params to device %s' % (str(self.device),))
        self.net = self.net.to(self.device)
        self.net = self.net.eval()

        self.get_logger().info('Moved model params to device %s' % (str(self.device),))
        self.image_buffer = RB(self.net.context_length,dtype=(float,(3,66,200)))
        self.s_torch = torch.linspace(0.0,1.0,steps=self.num_sample_points, dtype=self.dtype, device=self.device).unsqueeze(0)

        self.bezierM = mu.bezierM(self.s_torch, self.net.bezier_order)
        self.bezierMderiv = mu.bezierM(self.s_torch, self.net.bezier_order-1)
        self.bezierM2ndderiv = mu.bezierM(self.s_torch, self.net.bezier_order-2)
        self.bezierM.requires_grad = False
        self.image_sub = self.create_subscription( Image, '/cropped_publisher/images', self.addToBuffer, 1)
        boundary_bezier_order = 19
        
        self.inner_boundary = None
        self.ib_kdtree = None
        self.ib_sub = self.create_subscription(PointCloud2, "/inner_track_boundary/pcl", self.innerBoundaryCB, 1)
        self.ib_spline = None
        self.ib_tangent_spline = None
        self.ib_d = None

        self.outer_boundary = None
        self.ob_kdtree = None
        self.ob_sub = self.create_subscription(PointCloud2, "/outer_track_boundary/pcl", self.outerBoundaryCB, 1)
        self.ob_spline = None
        self.ob_tangent_spline = None
        self.ob_d = None

        self.boundary_loss = BoundaryLoss(time_reduction="all", batch_reduction="all", relu_type="Leaky", alpha=1.0, beta=1.0).type(self.dtype).to(self.device)
   
        num_optim_steps_param : Parameter = self.declare_parameter("num_optim_steps", value=-1)
        self.num_optim_steps = num_optim_steps_param.get_parameter_value().integer_value

        optim_step_size_param : Parameter = self.declare_parameter("optim_step_size", value=-1.0)
        self.optim_step_size = optim_step_size_param.get_parameter_value().double_value
  
        publish_curve_comparisons_param : Parameter = self.declare_parameter("publish_curve_comparisons", value=True)
        self.publish_curve_comparisons = publish_curve_comparisons_param.get_parameter_value().bool_value

        num_particles_param : Parameter = self.declare_parameter("num_particles", value=128)
        self.num_particles = num_particles_param.get_parameter_value().integer_value

        gaussian_filtering_param : Parameter = self.declare_parameter("gaussian_filtering", value=True)
        self.gaussian_filtering = gaussian_filtering_param.get_parameter_value().bool_value

        include_sample_curves_param : Parameter = self.declare_parameter("include_sample_curves", value=False)
        self.include_sample_curves = include_sample_curves_param.get_parameter_value().bool_value

        
        self.tf2_buffer : tf2_ros.Buffer = tf2_ros.Buffer(cache_time = Duration(seconds=15), node=self)
        self.tf2_listener : tf2_ros.TransformListener = tf2_ros.TransformListener(self.tf2_buffer, self, spin_thread=False)
      
        self.previous_boundaries : torch.Tensor = None
        self.current_particles : torch.Tensor = None




    def innerBoundaryCB(self, pc_msg: PointCloud2):
        l = list(deepracing_ros.convert.pointCloud2ToNumpy(pc_msg, field_names=["x","y","z","intensity"]))
        if len(l)<=0:
            return
        inner_boundary = np.array(l)
        inner_boundary_dists = inner_boundary[:,3]
        inner_boundary = inner_boundary[:,0:3]
        self.ib_d = inner_boundary_dists
        self.ib_spline : scipy.interpolate.BSpline = make_interp_spline(inner_boundary_dists, inner_boundary)
        self.ib_tangent_spline : scipy.interpolate.BSpline = self.ib_spline.derivative()
       # print(inner_boundary)
        if inner_boundary.shape[0]==0:
            return
        self.destroy_subscription(self.ib_sub)
        self.ib_sub = None
        self.inner_boundary = torch.cat([torch.from_numpy(inner_boundary.copy()).type(self.dtype).to(self.device), torch.ones(inner_boundary.shape[0], 1, dtype=self.dtype, device=self.device)], dim=1)
        self.ib_kdtree = KDTree(inner_boundary)
        print(self.inner_boundary)
    def outerBoundaryCB(self, pc_msg: PointCloud2):
        l = list(deepracing_ros.convert.pointCloud2ToNumpy(pc_msg, field_names=["x","y","z","intensity"]))
        if len(l)<=0:
            return
        outer_boundary = np.array(l)
        outer_boundary_dists = outer_boundary[:,3]
        outer_boundary = outer_boundary[:,0:3]
        self.ob_d = outer_boundary_dists
        self.ob_spline : scipy.interpolate.BSpline = make_interp_spline(outer_boundary_dists, outer_boundary)
        self.ob_tangent_spline : scipy.interpolate.BSpline = self.ob_spline.derivative()
      #  print(outer_boundary)
        if outer_boundary.shape[0]==0:
            return
        self.destroy_subscription(self.ob_sub)
        self.ob_sub = None
        self.outer_boundary = torch.cat([torch.from_numpy(outer_boundary.copy()).type(self.dtype).to(self.device), torch.ones(outer_boundary.shape[0], 1, dtype=self.dtype, device=self.device)], dim=1)
        self.ob_kdtree = KDTree(outer_boundary)
        print(self.outer_boundary)
    def addToBuffer(self, img_msg):
        imnp = self.cvbridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8") 
        if imnp.shape[0]<=0 or imnp.shape[1]<=0 or (not imnp.shape[2]==3) :
            return
        imtorch = tf.functional.to_tensor(imnp.copy())
        imnpdouble = imtorch.double().numpy()
        self.image_buffer.append(imnpdouble)
    def getTrackBounds(self, current_transform : torch.Tensor, lookahead : float, N=4000):
        # current_pos = current_transform[0:3,3]#.cpu().numpy()
        # current_pos_np = current_pos.cpu().numpy()
        current_transform_inv = torch.inverse(current_transform)
        current_position = current_transform_inv[0:3,3]

        bounds_global = torch.stack([self.outer_boundary, self.inner_boundary], dim=0)
        bounds_diffs = bounds_global[:,:,0:3] - current_position

        current_diff_norms = torch.norm(bounds_diffs, p=2, dim=2)
        ob_closest_idx = torch.argmin(current_diff_norms[0])
        ob_d0 = self.ib_d[ob_closest_idx]
        obdsamp = np.linspace(ob_d0 - 15.0, ob_d0 + lookahead, num=N)%(self.ob_d[-1])
        obtangents_global = torch.as_tensor(self.ob_tangent_spline(obdsamp), dtype=self.dtype, device=self.device)
      #  obtangents_global = obtangents_global/torch.norm(obtangents_global, dim=1, p=2)[:,None]
        obsamp_global = torch.as_tensor(self.ob_spline(obdsamp), dtype=self.dtype, device=self.device)
        obsamp_global = torch.cat([obsamp_global, torch.ones_like(obsamp_global[:,0]).unsqueeze(1)], dim=1)


        ib_closest_idx = torch.argmin(current_diff_norms[1])
        ib_d0 = self.ib_d[ib_closest_idx]
        ibdsamp = np.linspace(ib_d0 - 15.0, ib_d0 + lookahead, num=N)%(self.ib_d[-1])
        ibtangents_global = torch.as_tensor(self.ib_tangent_spline(ibdsamp), dtype=self.dtype, device=self.device)
        #ibtangents_global = ibtangents_global/torch.norm(ibtangents_global, dim=1, p=2)[:,None]
        ibsamp_global = torch.as_tensor(self.ib_spline(ibdsamp), dtype=self.dtype, device=self.device)
        ibsamp_global = torch.cat([ibsamp_global, torch.ones_like(ibsamp_global[:,0]).unsqueeze(1)], dim=1)
        

        bounds_global = torch.stack([obsamp_global, ibsamp_global], dim=0)
        bounds_local = torch.matmul(bounds_global, current_transform.t())[:,:,[0,2]]
        tangents_global = torch.stack([obtangents_global, ibtangents_global], dim=0)
        tangents_local = torch.matmul(tangents_global, current_transform[0:3,0:3].t())[:,:,[0,2]]

        normals_local = tangents_local.flip(dims=[2])
        normals_local[1]*=-1.0
        normals_local = normals_local/torch.norm(normals_local, p=2, dim=2)[:,:,None]

        return bounds_local, normals_local

    def getTrajectory(self):
        if self.pose_semaphore.acquire(timeout=1.0):
            if self.current_pose is None:
                self.pose_semaphore.release()
                self.get_logger().error("Haven't received pose message yet")
                return None, None, None
            current_pose_msg = deepcopy(self.current_pose)
            self.pose_semaphore.release()
        else:
            self.get_logger().error("Unable to acquire pose semaphore")
        if self.velocity_semaphore.acquire(timeout=1.0):
            if self.current_velocity is None:
                self.velocity_semaphore.release()
                self.get_logger().error("Haven't received twist message yet")
                return None, None, None
            current_twist_msg = deepcopy(self.current_velocity)
            self.velocity_semaphore.release()
        else:
            self.get_logger().error("Unable to acquire velocity semaphore")
            return None, None, None
        if (self.ib_kdtree is None) or (self.ob_kdtree is None):
            self.get_logger().error("Haven't gotten track bounds yet.")
            return None, None, None
        current_speed = np.linalg.norm(np.asarray([current_twist_msg.twist.linear.x, current_twist_msg.twist.linear.y, current_twist_msg.twist.linear.z ]))
        stamp = current_pose_msg.header.stamp
        # t = Time.from_msg(stamp)
        t = Time()
        transform_msg : TransformStamped = self.tf2_buffer.lookup_transform("base_link", deepracing_ros.world_coordinate_name, t, timeout=Duration(seconds=1))
        base_link_transform_msg : TransformStamped = self.tf2_buffer.lookup_transform("base_link", deepracing_ros.car_coordinate_name, t, timeout=Duration(seconds=1))
        transform = deepracing_ros.convert.transformMsgToTorch(transform_msg.transform, dtype=self.dtype, device=self.device)
        base_link_transform = deepracing_ros.convert.transformMsgToTorch(base_link_transform_msg.transform, dtype=self.dtype, device=self.device)
        
        
        imnp = np.array(self.image_buffer).copy()
        with torch.no_grad():
            imtorch = torch.from_numpy(imnp)
            imtorch.required_grad = False
            if ( not imtorch.shape[0] == self.net.context_length ):
                self.get_logger().error("%d images in the buffer. exptected %d." % (imtorch.shape[0], self.net.context_length))
                return None, None, None
            inputtorch : torch.Tensor = imtorch.unsqueeze(0).type(self.dtype).to(self.device)
            bezier_control_points, varfactors, covarfactors = self.net(inputtorch)
        
            bezier_control_points[:,:,0]*=self.xscale_factor
            initial_curve_full = torch.stack([bezier_control_points[0,:,0] , torch.zeros_like(bezier_control_points[0,:,1]) , bezier_control_points[0,:,1] ], dim=1)     
         #   print("initial_curve_full.shape: " + str(initial_curve_full.shape))
            initial_curve_msg : BCMessage = deepracing_ros.convert.toBezierCurveMsg(initial_curve_full, Header(frame_id=deepracing_ros.car_coordinate_name, stamp = current_pose_msg.header.stamp))
            sample_curves : List[BCMessage] = []

            if self.gaussian_filtering:
                cvb = torch.zeros(bezier_control_points.shape[1], 2, 2, dtype = varfactors.dtype, device = varfactors.device)
                cvb[:,0,0]=2.5
                cvb[:,1,1]=2.5
                scale_tril = torch.sqrt(cvb)
                curve_distribution : dist.MultivariateNormal = dist.MultivariateNormal(bezier_control_points[0], scale_tril=scale_tril, validate_args=False)
                self.current_particles = curve_distribution.sample((self.num_particles,))
                if self.include_sample_curves:
                    particles_cpu = self.current_particles.cpu()
                    particles_cpu = torch.stack([particles_cpu[:,:,0], torch.zeros_like(particles_cpu[:,:,0]), particles_cpu[:,:,1]], dim=2)
                    for i in range(particles_cpu.shape[0]):
                        sample_curves.append(deepracing_ros.convert.toBezierCurveMsg(particles_cpu[i], initial_curve_msg.header))
            else:
                self.current_particles = bezier_control_points

            

            mean_points = torch.matmul(self.bezierM, bezier_control_points)
            _, mean_v_s = mu.bezierDerivative(bezier_control_points, M=self.bezierMderiv)
            mean_vs_norm = torch.norm(mean_v_s, p=2, dim=2)
            ds = torch.mean( self.s_torch[:,1:] - self.s_torch[:,:-1], dim=1 )
            meanlength = mu.simpson(mean_vs_norm, ds)[0].item()

            boundarypoints, boundarynormals = self.getTrackBounds(transform, meanlength+20.0, N=int(round(meanlength/.05)))
        
            obnormals = boundarynormals[0].unsqueeze(0)
            obpoints = boundarypoints[0].unsqueeze(0)
            ibnormals = boundarynormals[1].unsqueeze(0)
            ibpoints = boundarypoints[1].unsqueeze(0)

            dclear = -1.5
           # particles_good = self.current_particles[ibidx*obidx]
            particles_good = self.current_particles
            if particles_good.shape[0]>1:
                particle_points = torch.matmul(self.bezierM[0], particles_good)
                _, v_s = mu.bezierDerivative(particles_good, M=self.bezierMderiv[0])
                v_t=v_s/self.deltaT
                speeds = torch.norm(v_t,dim=2,p=2)
                speed_squares = torch.square(speeds)

                v_snorm = torch.norm(v_s,dim=2,p=2)
                v_snorm_squares = torch.sum(v_s*v_s, dim=2)
                v_snorm_cubes = v_snorm*v_snorm_squares

                _, a_s = mu.bezierDerivative(particles_good, M=self.bezierM2ndderiv.expand(particles_good.shape[0],-1,-1), order=2)
                a_t=a_s/(self.deltaT*self.deltaT)
                accel_norms = torch.norm(a_t, dim=2)

                a_snorm = torch.norm(a_s,dim=2,p=2)
                a_snorm_squares = torch.square(a_snorm)
                dots = torch.sum(v_s*a_s, dim=2)

                radii = v_snorm_cubes/torch.sqrt(v_snorm_squares*a_snorm_squares - torch.square(dots)) + 5E-1

                centripetal_accels = speed_squares/radii
                centripetal_accels[centripetal_accels!=centripetal_accels] = 0.0

                average_speeds = torch.mean(speeds,dim=1)
                max_average_speed = torch.max(average_speeds)
                speed_scores = F.softmax(1.25*average_speeds.double(), dim=0)
                speed_scores[speed_scores!=speed_scores] = 0.0

                ca_deltas = torch.relu(centripetal_accels[:,15:-15] - self.max_centripetal_acceleration)
                max_ca_deltas, _ = torch.max(ca_deltas, dim=1)
                ca_scores = torch.clip(torch.exp(-0.25*max_ca_deltas.double()), 1E-10, 1.0)

                _, ib_distances = self.boundary_loss(particle_points, ibpoints.expand(particle_points.shape[0], -1, -1), ibnormals.expand(particle_points.shape[0], -1, -1))
                ib_max_distances, _ = torch.max(ib_distances, dim=1)
                ib_max_distances=F.relu(ib_max_distances + 1.5)

                _, ob_distances = self.boundary_loss(particle_points, obpoints.expand(particle_points.shape[0], -1, -1), obnormals.expand(particle_points.shape[0], -1, -1))
                ob_max_distances, _ = torch.max(ob_distances, dim=1)
                ob_max_distances=F.relu(ob_max_distances + 1.5)

                ib_scores = torch.clip( torch.exp(-1.0*ib_max_distances.double()), 1E-10, 1.0)
                ob_scores = torch.clip( torch.exp(-1.0*ob_max_distances.double()), 1E-10, 1.0)
                
                #score_products = ib_scores*ob_scores*ca_scores
                score_products = ca_scores*speed_scores*ib_scores*ob_scores#*accel_scores

                prefilterspeed = torch.mean(average_speeds)
                probs = (score_products/torch.sum(score_products))#.cpu().numpy()
                mean_particle = torch.sum(probs[:,None,None]*particles_good.double(), dim=0, keepdim=True).type(self.bezierM.dtype)
                x_samp = torch.matmul(self.bezierM, mean_particle)[0]
                _, vels = mu.bezierDerivative(mean_particle, M=self.bezierMderiv)
                vels=vels[0]/self.deltaT
                mean_particle = mean_particle[0]
                filtered_curve_full = torch.stack([ mean_particle[:,0] , torch.zeros_like(mean_particle[:,1]) , mean_particle[:,1] ] , dim=1)   
             #   print("filtered_curve_full.shape: " + str(filtered_curve_full.shape))
                filtered_curve_msg : BCMessage = deepracing_ros.convert.toBezierCurveMsg(filtered_curve_full, initial_curve_msg.header)

            else:
                x_samp = torch.matmul(self.bezierM, particles_good)[0]
                _, vels = mu.bezierDerivative(particles_good, M=self.bezierMderiv)
                vels=vels[0]/self.deltaT
                filtered_curve_msg : BCMessage = initial_curve_msg

            comparison : TrajComparison = TrajComparison(ego_pose = current_pose_msg, initial_curve = initial_curve_msg, filtered_curve = filtered_curve_msg, sample_curves = sample_curves)
            self.comparison_publisher.publish(comparison)

            speeds = torch.norm(vels, dim=1)
            distances_forward = mu.integrate.cumtrapz(speeds.unsqueeze(0), self.s_torch, initial=torch.zeros(1,1,dtype=vels.dtype,device=vels.device))[0]

            
            zeros = torch.zeros_like(x_samp[:,0])
            ones = torch.ones_like(zeros)
            
            positions = torch.stack([x_samp[:,0], zeros, x_samp[:,1],ones], dim=1)
            velocities = torch.stack([vels[:,0], zeros, vels[:,1]], dim=1)
            positions = torch.matmul(positions, base_link_transform.t())[:,0:3]
            velocities = torch.matmul(velocities, base_link_transform[0:3,0:3].t())
        return positions, velocities, distances_forward
        