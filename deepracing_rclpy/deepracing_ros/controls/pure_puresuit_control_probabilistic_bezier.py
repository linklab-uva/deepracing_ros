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
from deepracing_msgs.msg import PathRaw, ImageWithPath, BezierCurve as BCMessage, TrajComparison
from geometry_msgs.msg import Vector3Stamped, Vector3, PointStamped, Point, PoseStamped, Pose, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Header
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter
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



class ProbabilisticBezierPurePursuitControllerROS(PPC):
    def __init__(self):
        super(ProbabilisticBezierPurePursuitControllerROS, self).__init__()
        self.path_publisher = self.create_publisher(TrajComparison, "/predicted_paths", 1)
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
            self.net.double()
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
        self.bezierM.requires_grad = False

        if use_compressed_images_param.get_parameter_value().bool_value:
            self.image_sub = self.create_subscription( CompressedImage, '/cropped_publisher/images/compressed', self.addToBuffer, 1)
        else:
            self.image_sub = self.create_subscription( Image, '/cropped_publisher/images', self.addToBuffer, 1)

        self.Mboundaryfit = mu.bezierM( torch.linspace(0.0,1.0,steps=400,dtype=self.dtype,device=self.device).unsqueeze(0).repeat(2,1), 7)
        self.Mboundaryeval = mu.bezierM( torch.linspace(0.0,1.0,steps=10000,dtype=self.dtype,device=self.device).unsqueeze(0).repeat(2,1), 7)
        self.Mboundarytangenteval = mu.bezierM( torch.linspace(0.0,1.0,steps=self.Mboundaryeval.shape[1],dtype=self.dtype,device=self.device).unsqueeze(0).repeat(2,1), self.Mboundaryeval.shape[2]-2)
        # self.Mboundarynormaleval = mu.bezierM( torch.linspace(0.0,1.0,steps=4000,dtype=self.dtype,device=self.device).unsqueeze(0).repeat(2,1), 3)
        
        self.inner_boundary = None
        self.ib_kdtree = None
        self.ib_sub = self.create_subscription(PointCloud2, "/inner_track_boundary/pcl", self.innerBoundaryCB, 1)

        self.outer_boundary = None
        self.ob_kdtree = None
        self.ob_sub = self.create_subscription(PointCloud2, "/outer_track_boundary/pcl", self.outerBoundaryCB, 1)

        self.boundary_loss = BoundaryLoss(time_reduction="all", batch_reduction="all", relu_type="Leaky", alpha=5.0, beta=0.1).type(self.dtype).to(self.device)
   
        num_optim_steps_param : Parameter = self.declare_parameter("num_optim_steps", value=-1)
        self.num_optim_steps = num_optim_steps_param.get_parameter_value().integer_value

        optim_step_size_param : Parameter = self.declare_parameter("optim_step_size", value=-1.0)
        self.optim_step_size = optim_step_size_param.get_parameter_value().double_value
  
        publish_curve_comparisons_param : Parameter = self.declare_parameter("publish_curve_comparisons", value=True)
        self.publish_curve_comparisons = publish_curve_comparisons_param.get_parameter_value().bool_value

        
        self.tf2_buffer : tf2_ros.Buffer = tf2_ros.Buffer(node=self)
        self.tf2_listener : tf2_ros.TransformListener = tf2_ros.TransformListener(self.tf2_buffer, self, spin_thread=False)



    def innerBoundaryCB(self, pc_msg: PointCloud2):
        inner_boundary = np.array(list(deepracing_ros.convert.pointCloud2ToNumpy(pc_msg, field_names=["x","y","z"])))
       # print(inner_boundary)
        if inner_boundary.shape[0]==0:
            return
        self.destroy_subscription(self.ib_sub)
        self.ib_sub = None
        self.inner_boundary = torch.cat([torch.from_numpy(inner_boundary.copy()).type(self.dtype).to(self.device), torch.ones(inner_boundary.shape[0], 1, dtype=self.dtype, device=self.device)], dim=1)
        self.ib_kdtree = KDTree(inner_boundary)
        print(self.inner_boundary)
    def outerBoundaryCB(self, pc_msg: PointCloud2):
        outer_boundary = np.array(list(deepracing_ros.convert.pointCloud2ToNumpy(pc_msg, field_names=["x","y","z"])))
      #  print(outer_boundary)
        if outer_boundary.shape[0]==0:
            return
        self.destroy_subscription(self.ob_sub)
        self.ob_sub = None
        self.outer_boundary = torch.cat([torch.from_numpy(outer_boundary.copy()).type(self.dtype).to(self.device), torch.ones(outer_boundary.shape[0], 1, dtype=self.dtype, device=self.device)], dim=1)
        self.ob_kdtree = KDTree(outer_boundary)
        print(self.outer_boundary)
    def addToBuffer(self, img_msg):
        try:
            if isinstance(img_msg,CompressedImage):
                imnp = self.cvbridge.compressed_imgmsg_to_cv2(img_msg, desired_encoding="rgb8") 
            elif isinstance(img_msg,Image):
                imnp = self.cvbridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8") 
            else:
                raise ValueError( "Invalid type %s passed to addToBuffer" % (str(type(img_msg)),) )
        except ValueError as e:
            raise e
        except Exception as e:
            return
        if imnp.shape[0]<=0 or imnp.shape[1]<=0 or (not imnp.shape[2]==3) :
            return
        
        imnpdouble = tf.functional.to_tensor(imnp.copy()).double().numpy()
        self.image_buffer.append(imnpdouble)

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
            return None, None, None
        current_pm = deepracing_ros.convert.poseMsgToTorch(current_pose_msg.pose, dtype=self.dtype, device=self.device)
        current_pm_inv = torch.inverse(current_pm)
        stamp = current_pose_msg.header.stamp
        imnp = np.array(self.image_buffer).copy()
        with torch.no_grad():
            imtorch = torch.from_numpy(imnp)
            imtorch.required_grad = False
            if ( not imtorch.shape[0] == self.net.context_length ):
                return None, None, None
            inputtorch : torch.Tensor = imtorch.unsqueeze(0).type(self.dtype).to(self.device)
            bezier_control_points, varfactors, covarfactors = self.net(inputtorch)
            scale_trils = torch.diag_embed(varfactors[0]) + torch.diag_embed(covarfactors[0], offset=-1)
            covariances = torch.matmul(scale_trils, scale_trils.transpose(1,2))
            covarsmsg = torch.zeros(covariances.shape[0], 3, 3, dtype=self.dtype, device=self.device)
            covarsmsg[:,0,0] = covariances[:,0,0]
            covarsmsg[:,1,1] =  1E-7
            covarsmsg[:,2,2] =  covariances[:,1,1]
            covarsmsg[:,2,0] = covarsmsg[:,0,2] = covariances[:,1,0]
            covarsmsg=covarsmsg.cpu()
         #   print(covarsmsg)
        paths_msg : TrajComparison = TrajComparison(ego_pose=current_pose_msg)
        initial_guess = bezier_control_points[0].detach().clone()
        initial_guess[:,0]*=self.xscale_factor
        igcpu = initial_guess.cpu()
        igcpu = torch.stack([ igcpu[:,0], torch.zeros_like(igcpu[:,0]), igcpu[:,1] ], dim=1)
        paths_msg.curves.append(deepracing_ros.convert.toBezierCurveMsg(igcpu, Header(frame_id=deepracing_ros.car_coordinate_name, stamp=stamp), covars=covarsmsg))
        paths_msg.optimization_time=-1.0

        if self.num_optim_steps>0 and (self.ib_kdtree is not None) and (self.ob_kdtree is not None):
            #print("Running optimization")
            tick = time.time()
            current_pos = current_pm[0:3,3]#.cpu().numpy()
            current_pos_np = current_pos.cpu().numpy()

            ib_closest_delta, ib_closest_idx = self.ib_kdtree.query(current_pos_np)
            ibstart = ib_closest_idx - 40
            ibend = ib_closest_idx + 360
            ibidx = torch.arange(ibstart,ibend, step=1, dtype=torch.int64)%self.inner_boundary.shape[0]
            ibsamp = torch.matmul(self.inner_boundary[ibidx], current_pm_inv.t())[:,[0,2]]
            
            ob_closest_delta, ob_closest_idx = self.ob_kdtree.query(current_pos_np)
            obstart = ob_closest_idx - 40
            obend = ob_closest_idx + 360
            obidx = torch.arange(obstart,obend, step=1, dtype=torch.int64)%self.outer_boundary.shape[0]
            obsamp = torch.matmul(self.outer_boundary[obidx], current_pm_inv.t())[:,[0,2]]

            boundaries = torch.stack([obsamp, ibsamp], dim=0)

            _, boundarycurveslocal = mu.bezierLsqfit(boundaries, 7, M=self.Mboundaryfit)

            if boundarycurveslocal.device==torch.device("cpu"):
                bcxz = boundarycurveslocal.clone()
            else:
                bcxz = boundarycurveslocal.cpu()
            boundarycurvesformsg = torch.stack([bcxz[:,:,0], current_pos_np[1]*torch.ones_like(bcxz[:,:,0]), bcxz[:,:,1]], dim=2)

            
            boundarypoints = torch.matmul(self.Mboundaryeval, boundarycurveslocal)
            _, boundarytangents = mu.bezierDerivative(boundarycurveslocal, M=self.Mboundarytangenteval)
            boundarytangents = boundarytangents/torch.norm(boundarytangents, dim=2)[:,:,None]
            boundarynormals = boundarytangents.clone().flip(dims=[2])
            boundarynormals[1,:,0]*=-1.0

            obnormals = boundarynormals[0].unsqueeze(0)
            obpoints = boundarypoints[0].unsqueeze(0) - 1.5*obnormals

            
            ibnormals = boundarynormals[1].unsqueeze(0)
            ibpoints = boundarypoints[1].unsqueeze(0) - 1.5*ibnormals

            paths_msg.outer_boundary_curve = deepracing_ros.convert.toBezierCurveMsg(boundarycurvesformsg[0], Header(frame_id=deepracing_ros.car_coordinate_name, stamp=stamp))
            paths_msg.inner_boundary_curve = deepracing_ros.convert.toBezierCurveMsg(boundarycurvesformsg[1], Header(frame_id=deepracing_ros.car_coordinate_name, stamp=stamp))

            evalpoints = torch.matmul(self.bezierM, initial_guess.unsqueeze(0))
            initial_guess_points = evalpoints.clone()

            mask=[False] + [True for asdf in range(bezier_control_points.shape[1]-1)]
            bcmodel = mu.BezierCurveModule(initial_guess, mask=mask).train()
            dT = self.deltaT
            dT2 = dT*dT
            maxacent = 9.8*3
            maxalinear = 9.8*2
            optimizer = SGD(bcmodel.parameters(), lr=self.optim_step_size, momentum=0.0)
            i = 0
            maxloss = 1.0
            while maxloss>0.0 and (i<self.num_optim_steps):
            #    print("Step %d" %(i+1,))
                all_control_points = bcmodel.allControlPoints()
                # print(self.bezierM.shape)
                # print(all_control_points.shape)
                evalpoints = torch.matmul(self.bezierM, all_control_points)
                
                stepfactor = np.sqrt((self.num_optim_steps-i)/self.num_optim_steps)
                # stepfactor = 1.0
                _, l1 = self.boundary_loss(evalpoints, ibpoints, ibnormals)
                _, l2 = self.boundary_loss(evalpoints, obpoints, obnormals)
                l = torch.cat([l1,l2],dim=0)
                lmax, idxmax = torch.max(l,dim=0)
                maxloss = torch.max(lmax)
               # print(stepfactor)
                optimizer.zero_grad()
                loss = stepfactor*maxloss
                loss.backward()
                optimizer.step()
                i+=1
                currentcurvecpu = all_control_points[0].detach().cpu()
                currentcurvecpu = torch.stack([ currentcurvecpu[:,0], torch.zeros_like(currentcurvecpu[:,0]), currentcurvecpu[:,1] ], dim=1)
                paths_msg.curves.append(deepracing_ros.convert.toBezierCurveMsg(currentcurvecpu, Header(frame_id=deepracing_ros.car_coordinate_name, stamp=stamp), covars=covarsmsg))
            output = bcmodel.allControlPoints().detach()
            diffs = evalpoints - initial_guess_points
            diffnorms = torch.norm(diffs, p=2, dim=2)[0]
            if torch.all(output==output) and torch.all(diffs==diffs) and torch.max(diffnorms)<10.0:
                bezier_control_points = output
                paths_msg.optimization_succeeded = True
            else:
                paths_msg.optimization_succeeded = False
            tock = time.time()
            paths_msg.optimization_time = float(tock-tick)
       # print(bezier_control_points.shape)
        finalcurvecpu = bezier_control_points[0].cpu()
        finalcurvecpu = torch.stack([ finalcurvecpu[:,0], torch.zeros_like(finalcurvecpu[:,0]), finalcurvecpu[:,1] ], dim=1)
        paths_msg.curves.append(deepracing_ros.convert.toBezierCurveMsg(finalcurvecpu, Header(frame_id=deepracing_ros.car_coordinate_name, stamp=stamp), covars=covarsmsg))
        if self.publish_curve_comparisons:
            self.path_publisher.publish(paths_msg)
            
        with torch.no_grad():
            evalpoints = torch.matmul(self.bezierM, bezier_control_points)
            x_samp = evalpoints[0]
            x_samp[:,1]-=self.z_offset
            _, predicted_tangents = mu.bezierDerivative(bezier_control_points, M = self.bezierMderiv, order=1)
            predicted_tangent_norms = torch.norm(predicted_tangents, p=2, dim=2)
            v_t = self.velocity_scale_factor*(1.0/self.deltaT)*predicted_tangents[0]
            distances_forward = mu.integrate.cumtrapz(predicted_tangent_norms, self.s_torch, initial=torch.zeros(1,1,dtype=v_t.dtype,device=v_t.device))[0]
            vels = v_t
        zeros = torch.zeros_like(x_samp[:,0])
        positions = torch.stack([x_samp[:,0], zeros, x_samp[:,1]], dim=1)
        velocities = torch.stack([vels[:,0], zeros, vels[:,1]], dim=1)
        return positions, velocities, None
        