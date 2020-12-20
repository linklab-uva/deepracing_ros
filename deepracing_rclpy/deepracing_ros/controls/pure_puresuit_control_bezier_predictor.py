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
import deepracing_ros.convert
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image, CompressedImage, PointCloud2
from deepracing_msgs.msg import PathRaw, ImageWithPath, BezierCurve as BCMessage, TrajComparison
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
import cv2, numpy as np
import timeit
import array
import torch.nn.functional as F
from scipy.spatial.kdtree import KDTree
from copy import deepcopy

from scipy.interpolate import BSpline, make_interp_spline

from copy import deepcopy

def npTrajectoryToROS(trajectory : np.ndarray, velocities : np.ndarray, frame_id = "map"):
    rtn : Path = Path()
    rtn.header.frame_id = frame_id
    for i in range(trajectory.shape[0]):
        point = trajectory[i]
        forward = np.array((velocities[i,0],0.0,velocities[i,1]), dtype=np.float64)
        up = np.array((0.0,1.0,0.0), dtype=np.float64)
        left = np.cross(up,forward)
        left[2] = 0.0
        left = left / la.norm(left)
        trueup =  np.cross(forward,left)
        trueup = trueup / la.norm(trueup)

        posestamped : PoseStamped = PoseStamped()
        posestamped.header.frame_id = frame_id
        pose : Pose = Pose()
        pose.position.x = point[0]
        pose.position.z = point[1]
       # pose.position.y = 0
       # pose.position.z = 0
        r = Rot.from_matrix(np.vstack((left, trueup, forward)).transpose())
        quat = r.as_quat()
        pose.orientation = Quaternion()
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        posestamped.pose = pose
        rtn.poses.append(posestamped)
    return rtn
class AdmiralNetBezierPurePursuitControllerROS(PPC):
    def __init__(self):
        super(AdmiralNetBezierPurePursuitControllerROS, self).__init__()

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
        use_float = config.get("use_float",False)
        bezier_order = config.get("bezier_order",None)
        sequence_length = config.get("sequence_length",None)
        use_3dconv = config.get("use_3dconv",True)
        self.fix_first_point = config["fix_first_point"]
        self.rosclock = ROSClock()
        with rpyutils.add_dll_directories_from_env("PATH"):
            self.cvbridge : cv_bridge.CvBridge = cv_bridge.CvBridge()
        self.bufferdtpub = self.create_publisher(Float64, "/buffer_dt", 1)
        #self.rosclock._set_ros_time_is_active(True)



        plot_param : Parameter = self.declare_parameter("plot", value=False)
        self.plot : bool = plot_param.get_parameter_value().bool_value
        if(self.plot):
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

        
        crop_origin_param : Parameter = self.declare_parameter("crop_origin", value=[-1, -1])
        self.crop_origin = list(crop_origin_param.get_parameter_value().integer_array_value)

        crop_size_param : Parameter = self.declare_parameter("crop_size", value=[-1, -1])
        self.crop_size  = list(crop_size_param.get_parameter_value().integer_array_value)


       
        
        
        self.net : NN.Module = M.AdmiralNetCurvePredictor(context_length= context_length, input_channels=input_channels, params_per_dimension=bezier_order+1-int(self.fix_first_point), use_3dconv=use_3dconv) 
        if use_float:
            self.dtype = torch.float32
            self.net = self.net.float()
        else:
            self.dtype = torch.float64
            self.net.double()
        self.get_logger().info('Loading model file: %s' % (model_file) )
        self.net.load_state_dict(torch.load(model_file,map_location=torch.device("cpu")))
        self.get_logger().info('Loaded model file: %s' % (model_file) )
        self.get_logger().info('Moving model params to device %s' % (str(self.device),))
        self.net = self.net.to(self.device)
        self.net = self.net.eval()
        self.ib_viol_counter=1
        self.ob_viol_counter=1

        self.get_logger().info('Moved model params to device %s' % (str(self.device),))
        self.image_buffer = RB(self.net.context_length,dtype=(float,(3,66,200)))
        self.s_np = np.linspace(0,1,self.num_sample_points)
        self.s_torch = torch.from_numpy(self.s_np.copy()).unsqueeze(0).type(self.dtype).to(self.device)
        self.bezier_order = self.net.params_per_dimension-1+int(self.fix_first_point)
        self.bezierM = mu.bezierM(self.s_torch,self.bezier_order)#.type(dtype).to(self.device)
        self.bezierMderiv = mu.bezierM(self.s_torch,self.bezier_order-1)
        self.bezierM2ndderiv = mu.bezierM(self.s_torch,self.bezier_order-2)
        self.buffertimer = timeit.Timer(stmt=self.addToBuffer)
        if self.fix_first_point:
            self.initial_zeros = torch.zeros(1,1,2, device=self.device, dtype=self.dtype)
        self.bezierM.requires_grad = False
      #  self.bezierMderiv.requires_grad = False
        self.bezierM2ndderiv.requires_grad = False
        if use_compressed_images_param.get_parameter_value().bool_value:
            self.image_sub = self.create_subscription( CompressedImage, '/cropped_publisher/images/compressed', self.addToBuffer, 1)
        else:
            self.image_sub = self.create_subscription( Image, '/cropped_publisher/images', self.addToBuffer, 1)

        self.Mboundaryfit = mu.bezierM( torch.linspace(0.0,1.0,steps=440,dtype=self.dtype,device=self.device).unsqueeze(0).repeat(2,1), 7)
        self.Mboundaryeval = mu.bezierM( torch.linspace(0.0,1.0,steps=5000,dtype=self.dtype,device=self.device).unsqueeze(0).repeat(2,1), 7)
        self.Mboundarytangenteval = mu.bezierM( torch.linspace(0.0,1.0,steps=5000,dtype=self.dtype,device=self.device).unsqueeze(0).repeat(2,1), 6)
        # self.Mboundarynormaleval = mu.bezierM( torch.linspace(0.0,1.0,steps=4000,dtype=self.dtype,device=self.device).unsqueeze(0).repeat(2,1), 3)
        
        self.inner_boundary = None
        self.ib_kdtree = None
        self.ib_sub = self.create_subscription(PointCloud2, "/inner_track_boundary/pcl", self.innerBoundaryCB, 1)

        self.outer_boundary = None
        self.ob_kdtree = None
        self.ob_sub = self.create_subscription(PointCloud2, "/outer_track_boundary/pcl", self.outerBoundaryCB, 1)

        self.boundary_loss = BoundaryLoss(time_reduction="max", alpha=0.5, beta=0.0).type(self.dtype).to(self.device)



    def innerBoundaryCB(self, pc_msg: PointCloud2):
        inner_boundary = np.array(list(deepracing_ros.convert.pointCloud2ToNumpy(pc_msg, field_names=["x","y","z"])))
        self.ib_sub.destroy()
        self.ib_sub = None
        self.inner_boundary = torch.from_numpy(inner_boundary.copy()).type(self.dtype).to(self.device)
        self.ib_kdtree = KDTree(inner_boundary)
        print(self.inner_boundary)
    def outerBoundaryCB(self, pc_msg: PointCloud2):
        outer_boundary = np.array(list(deepracing_ros.convert.pointCloud2ToNumpy(pc_msg, field_names=["x","y","z"])))
        self.ob_sub.destroy()
        self.ob_sub = None
        self.outer_boundary = torch.from_numpy(outer_boundary.copy()).type(self.dtype).to(self.device)
        self.ob_kdtree = KDTree(outer_boundary)
        print(self.outer_boundary)
    def addToBuffer(self, img_msg):
        try:
            if isinstance(img_msg,CompressedImage):
                imnp = self.cvbridge.compressed_imgmsg_to_cv2(img_msg, desired_encoding="bgr8") 
            elif isinstance(img_msg,Image):
                imnp = self.cvbridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8") 
            else:
                raise ValueError( "Invalid type %s passed to addToBuffer" % (str(type(img_msg)),) )
        except ValueError as e:
            raise e
        except Exception as e:
            return
        if imnp.shape[0]<=0 or imnp.shape[1]<=0 or (not imnp.shape[2]==3) :
            return
        if self.crop_origin[0]>=0 and self.crop_origin[1]>=0:
            imcrop1 = imnp[self.crop_origin[1]:,self.crop_origin[0]:,:]
        else:
            imcrop1 = imnp
        if self.crop_size[0]>0 and self.crop_size[1]>0:
            # imcrop2 = imcrop1[0:self.crop_size[1],0:self.crop_size[0],:]
            imcrop2 = imcrop1[0:self.crop_size[1]]
        else:
            imcrop2 = imcrop1
        imnpdouble = tf.functional.to_tensor(cv2.cvtColor(deepracing.imutils.resizeImage( imcrop2.copy(), (66,200) ), cv2.COLOR_BGR2RGB ) ).double().numpy()
       # imnpdouble = tf.functional.to_tensor(cv2.cvtColor(deepracing.imutils.resizeImage( imnp.copy(), (66,200) ), cv2.COLOR_BGR2RGB ) ).double().numpy()
        self.image_buffer.append(imnpdouble)
    # def compressedImageCallback(self, img_msg : CompressedImage):
    #     self.addToBuffer(img_msg)
    # def imageCallback(self, img_msg : Image):
    #     self.addToBuffer(img_msg)
    def getTrajectory(self):
        # print(self.current_pose_mat)
        boundary_check = self.num_optim_steps>0
        if boundary_check and (self.current_pose_mat is None):
            return super().getTrajectory()
        elif boundary_check:
            if self.pose_semaphore.acquire(timeout=1.0):
                current_pose_msg = deepcopy(self.current_pose)
                current_pm = self.current_pose_mat.clone()
                self.pose_semaphore.release()
            else:
                return super().getTrajectory()
            current_pm = current_pm.type(self.dtype).to(self.device)
        stamp = current_pose_msg.header.stamp
        imnp = np.array(self.image_buffer).astype(np.float32).copy()
        with torch.no_grad():
            imtorch = torch.from_numpy(imnp.copy())
            imtorch.required_grad = False
            if ( not imtorch.shape[0] == self.net.context_length ):
                return super().getTrajectory()
            inputtorch : torch.Tensor = imtorch.unsqueeze(0).type(self.dtype).to(self.device)
            network_predictions = self.net(inputtorch)
            if self.fix_first_point:  
                bezier_control_points = torch.cat((self.initial_zeros,network_predictions.transpose(1,2)),dim=1)    
            else:
                bezier_control_points = network_predictions.transpose(1,2)
        paths_msg : TrajComparison = TrajComparison(ego_pose=current_pose_msg)
        initial_guess = bezier_control_points.detach().clone()
        paths_msg.curves.append(deepracing_ros.convert.toBezierCurveMsg(initial_guess[0].cpu().numpy(), Header(frame_id="car", stamp=stamp)))
        paths_msg.optimization_time=-1.0
        if boundary_check and (self.ib_kdtree is not None) and (self.ob_kdtree is not None):
            tick = time.time()
            current_pos = current_pm[0:3,3].cpu().numpy()
            _, ib_closest_idx = self.ib_kdtree.query(current_pos)
            ibstart, ibend = ib_closest_idx - 40, ib_closest_idx+400
            ibidx = torch.arange(ibstart,ibend, step=1, dtype=torch.int64)%self.inner_boundary.shape[0]
            ibsamp = self.inner_boundary[ibidx]#[:,[0,2]]
            
            _, ob_closest_idx = self.ob_kdtree.query(current_pos)
            obstart, obend = ob_closest_idx - 40, ob_closest_idx+400
            obidx = torch.arange(obstart,obend, step=1, dtype=torch.int64)%self.outer_boundary.shape[0]
            obsamp = self.outer_boundary[obidx]#[:,[0,2]]

            boundaries = torch.stack([obsamp, ibsamp], dim=0)

            _, boundarycurves = mu.bezierLsqfit(boundaries, 7, M=self.Mboundaryfit)
            boundarycurvesaug = torch.cat([boundarycurves,  torch.ones_like(boundarycurves[:,:,0]).unsqueeze(2)], dim=2).transpose(1,2)
            boundarycurveslocal = torch.matmul(torch.inverse(current_pm.to(self.device)), boundarycurvesaug)[:,0:3].transpose(1,2)[:,:,[0,2]]
            
            boundarypoints = torch.matmul(self.Mboundaryeval, boundarycurveslocal)
            _, boundarytangents = mu.bezierDerivative(boundarycurveslocal, M=self.Mboundarytangenteval)
            boundarytangents = boundarytangents/torch.norm(boundarytangents, dim=2)[:,:,None]
            boundarynormals = boundarytangents.clone().flip(dims=[2])
            boundarynormals[1,:,0]*=-1.0

            obnormals = boundarynormals[0].unsqueeze(0)
            obpoints = boundarypoints[0].unsqueeze(0) - 1.0*obnormals

            
            ibnormals = boundarynormals[1].unsqueeze(0)
            ibpoints = boundarypoints[1].unsqueeze(0) - 1.0*ibnormals

            
            paths_msg.outer_boundary_curve = deepracing_ros.convert.toBezierCurveMsg(boundarycurves[0].cpu().numpy(), Header(frame_id="car", stamp=stamp))
            paths_msg.inner_boundary_curve = deepracing_ros.convert.toBezierCurveMsg(boundarycurves[1].cpu().numpy(), Header(frame_id="car", stamp=stamp))

            mask=[True for asdf in range(bezier_control_points.shape[1])]
            mask[0]=not self.fix_first_point
            evalpoints = torch.matmul(self.bezierM, initial_guess)
            initial_guess_points = evalpoints.clone()
            bcmodel = mu.BezierCurveModule(initial_guess, mask=mask)
          #  bcmodel.train()
            dT = self.deltaT
            dT2 = dT*dT
            maxacent = 9.8*3
            maxalinear = 9.8*2
            _, l1 = self.boundary_loss(evalpoints, ibpoints, ibnormals)
            _, l2 = self.boundary_loss(evalpoints, obpoints, obnormals)
            optimizer = SGD(bcmodel.parameters(), lr=0.45, momentum=0.1)
            i = 0
            while (l1>0.0 or l2>0.0) and (i<self.num_optim_steps):
                all_control_points = bcmodel.allControlPoints()
                evalpoints = torch.matmul(self.bezierM, all_control_points)
                # _, v_s = mu.bezierDerivative(all_control_points, M=self.bezierMderiv)
                # v_t = v_s/dT
                # speeds = torch.norm(v_t, p=2, dim=2)
                # curvetangents = v_t/speeds[:,:,None]
                # speedsquares = torch.square(speeds)
                # speedcubes = speedsquares*speeds
                # _, a_s =  mu.bezierDerivative(all_control_points, M=self.bezierM2ndderiv, order=2)
                # a_t = a_s/dT2
                # accels = torch.norm(a_t, p=2, dim=2)
                # v_text = torch.cat([v_t, torch.zeros_like(v_t[:,:,0]).unsqueeze(2)], dim=2)
                # a_text = torch.cat([a_t, torch.zeros_like(a_t[:,:,0]).unsqueeze(2)], dim=2)
                # radii = (speedcubes/(torch.norm(torch.cross(v_text,a_text, dim=2), p=2, dim=2) + 1E-3))# + 1.0
                # angvels = speeds/radii
                # centriptelaccels = speedsquares/radii
                stepfactor = float(np.sqrt((self.num_optim_steps-i)/self.num_optim_steps))
                _, l1 = self.boundary_loss(evalpoints, ibpoints, ibnormals)
                _, l2 = self.boundary_loss(evalpoints, obpoints, obnormals)
               # print(stepfactor)
                optimizer.zero_grad()
                loss = stepfactor*(l1+l2)# - 0.5*stepfactor*torch.mean(speeds) + 0.1*stepfactor*torch.max(torch.nn.functional.relu(accels-maxalinear))
                loss.backward()
                optimizer.step()
                i+=1
                paths_msg.curves.append(deepracing_ros.convert.toBezierCurveMsg(all_control_points[0].detach().cpu().numpy(), Header(frame_id="car", stamp=stamp)))
            output = bcmodel.allControlPoints().detach()
            diffs = evalpoints - initial_guess_points
            diffnorms = torch.norm(diffs, p=2, dim=2)[0]
            if torch.all(output==output) and torch.all(diffs==diffs) and torch.max(diffnorms)<10.0:
                bezier_control_points = output
            tock = time.time()
            paths_msg.optimization_time = float(tock-tick)
        paths_msg.curves.append(deepracing_ros.convert.toBezierCurveMsg(bezier_control_points[0].cpu().numpy(), Header(frame_id="car", stamp=stamp)))
        self.path_publisher.publish(paths_msg)
            
 
        with torch.no_grad():
            evalpoints = torch.matmul(self.bezierM, bezier_control_points)
            x_samp = evalpoints[0]
            x_samp[:,0]*=self.xscale_factor
            x_samp[:,1]-=self.z_offset
            #x_samp_t = x_samp.transpose(0,1)


            

            _, predicted_tangents = mu.bezierDerivative(bezier_control_points, M = self.bezierMderiv, order=1)
            #predicted_tangents = predicted_tangents
            predicted_tangent_norms = torch.norm(predicted_tangents, p=2, dim=2)
            v_t = self.velocity_scale_factor*(1.0/self.deltaT)*predicted_tangents[0]
            distances_forward = mu.integrate.cumtrapz(predicted_tangent_norms, self.s_torch, initial=torch.zeros(1,1,dtype=v_t.dtype,device=v_t.device))[0]
        


            _, predicted_normals = mu.bezierDerivative(bezier_control_points, M = self.bezierM2ndderiv, order=2)
            predicted_normals = predicted_normals[0]
            predicted_normal_norms = torch.norm(predicted_normals, p=2, dim=1)
                           
              
            cross_prod_norms = torch.abs(predicted_tangents[0,:,0]*predicted_normals[:,1] - predicted_tangents[0,:,1]*predicted_normals[:,0])
            radii = torch.pow(predicted_tangent_norms[0],3) / cross_prod_norms
            speeds = self.max_speed*(torch.ones_like(radii)).type(self.dtype).to(self.device)
            centripetal_accelerations = torch.square(speeds)/radii
            max_allowable_speeds = torch.sqrt(self.max_centripetal_acceleration*radii)
            idx = centripetal_accelerations>self.max_centripetal_acceleration
            speeds[idx] = max_allowable_speeds[idx]
          #  vels = speeds[:,None]*(predicted_tangents[0]/predicted_tangent_norms[0,:,None])
            vels = v_t
            #distances_forward = torch.cat((torch.zeros(1, dtype=x_samp.dtype, device=x_samp.device), torch.cumsum(torch.norm(x_samp[1:]-x_samp[:-1],p=2,dim=1), 0)), dim=0)
        
        x_samp[:,1]-=self.z_offset
        return x_samp, vels, distances_forward
        