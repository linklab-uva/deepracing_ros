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
from deepracing_msgs.msg import PathRaw, ImageWithPath, BezierCurve as BCMessage
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

from scipy.interpolate import BSpline, make_interp_spline

from copy import deepcopy

class TrajOptimPurePursuitControllerROS(PPC):
    def __init__(self):
        super(TrajOptimPurePursuitControllerROS, self).__init__()

        self.rosclock = ROSClock()

        fix_first_point_param : Parameter = self.declare_parameter("fix_first_point", value=True)
        self.fix_first_point : bool = fix_first_point_param.get_parameter_value().bool_value

        use_float_param : Parameter = self.declare_parameter("use_float", value=True)
        self.use_float : bool = use_float_param.get_parameter_value().bool_value

        if self.use_float:
            self.dtype = torch.float32
        else:
            self.dtype = torch.float64

        bezier_order_param : Parameter = self.declare_parameter("bezier_order", value=7)
        self.bezier_order : int = bezier_order_param.get_parameter_value().integer_value

        deltaT_param : Parameter = self.declare_parameter("deltaT", value=1.5)
        self.deltaT : float = deltaT_param.get_parameter_value().double_value

        x_scale_factor_param : Parameter = self.declare_parameter("x_scale_factor", value=1.0)
        self.xscale_factor : float = x_scale_factor_param.get_parameter_value().double_value

        z_offset_param : Parameter = self.declare_parameter("z_offset", value=self.L/2.0)
        self.z_offset : float = z_offset_param.get_parameter_value().double_value


        velocity_scale_param : Parameter = self.declare_parameter("velocity_scale_factor", value=1.0)
        self.velocity_scale_factor : float = velocity_scale_param.get_parameter_value().double_value
        
        num_sample_points_param : Parameter = self.declare_parameter("num_sample_points", value=60)
        self.num_sample_points : int = num_sample_points_param.get_parameter_value().integer_value     
        
        self.s_torch = torch.linspace(0.0,1.0,steps=self.num_sample_points, dtype=self.dtype, device=self.device).unsqueeze(0)
        self.bezierM = mu.bezierM(self.s_torch,self.bezier_order)#.type(dtype).to(self.device)
        self.bezierMderiv = mu.bezierM(self.s_torch,self.bezier_order-1)
        self.bezierM2ndderiv = mu.bezierM(self.s_torch,self.bezier_order-2)

        self.Mboundaryfit = mu.bezierM( torch.linspace(0.0,1.0,steps=300,dtype=self.dtype,device=self.device).unsqueeze(0).repeat(2,1), 7)
        self.Mboundaryeval = mu.bezierM( torch.linspace(0.0,1.0,steps=10000,dtype=self.dtype,device=self.device).unsqueeze(0).repeat(2,1), 7)
        self.Mboundarytangenteval = mu.bezierM( torch.linspace(0.0,1.0,steps=10000,dtype=self.dtype,device=self.device).unsqueeze(0).repeat(2,1), 6)
        # self.Mboundarynormaleval = mu.bezierM( torch.linspace(0.0,1.0,steps=4000,dtype=self.dtype,device=self.device).unsqueeze(0).repeat(2,1), 3)
        
        self.inner_boundary = None
        self.ib_kdtree = None
        self.ib_sub = self.create_subscription(PointCloud2, "/inner_track_boundary/pcl", self.innerBoundaryCB, 1)

        self.outer_boundary = None
        self.ob_kdtree = None
        self.ob_sub = self.create_subscription(PointCloud2, "/outer_track_boundary/pcl", self.outerBoundaryCB, 1)

        self.boundary_loss = BoundaryLoss(time_reduction="max", alpha=2.0, beta=0.0).type(self.dtype).to(self.device)



    def innerBoundaryCB(self, pc_msg: PointCloud2):
        print("Got an inner boundary message")
        generator = deepracing_ros.convert.pointCloud2ToNumpy(pc_msg, field_names=["x","y","z"])
        print(generator)
        l = list(generator)
        print(l)
        inner_boundary = np.array(l)
        self.ib_sub.destroy()
        self.ib_sub = None
        self.inner_boundary = torch.from_numpy(inner_boundary.copy()).type(self.dtype).to(self.device)
        self.ib_kdtree = KDTree(inner_boundary)
        print(self.inner_boundary)
    def outerBoundaryCB(self, pc_msg: PointCloud2):
        print("Got an outer boundary message")
        outer_boundary = np.array(list(deepracing_ros.convert.pointCloud2ToNumpy(pc_msg, field_names=["x","y","z"])))
        self.ob_sub.destroy()
        self.ob_sub = None
        self.outer_boundary = torch.from_numpy(outer_boundary.copy()).type(self.dtype).to(self.device)
        self.ob_kdtree = KDTree(outer_boundary)
        print(self.outer_boundary)

    # def compressedImageCallback(self, img_msg : CompressedImage):
    #     self.addToBuffer(img_msg)
    # def imageCallback(self, img_msg : Image):
    #     self.addToBuffer(img_msg)
    def getTrajectory(self):
        # print(self.current_pose_mat)
        if (self.current_pose_mat is None) or (self.current_velocity is None) or (self.ib_kdtree is None) or (self.ob_kdtree is None):
            return super().getTrajectory()
        current_pm = self.current_pose_mat.clone().type(self.dtype).to(self.device)
        current_velocity = deepcopy(self.current_velocity)
        speed = float(np.linalg.norm(np.array([current_velocity.twist.linear.x, current_velocity.twist.linear.y, current_velocity.twist.linear.z])))
        print(speed)
        
        current_pos = current_pm[0:3,3].cpu().numpy()
        _, ib_closest_idx = self.ib_kdtree.query(current_pos)
        ibstart, ibend = ib_closest_idx - 20, ib_closest_idx+280
        ibidx = torch.arange(ibstart,ibend, step=1, dtype=torch.int64)%self.inner_boundary.shape[0]
        ibsamp = self.inner_boundary[ibidx]#[:,[0,2]]
        
        _, ob_closest_idx = self.ob_kdtree.query(current_pos)
        obstart, obend = ob_closest_idx - 20, ob_closest_idx+280
        obidx = torch.arange(obstart,obend, step=1, dtype=torch.int64)%self.outer_boundary.shape[0]
        obsamp = self.outer_boundary[obidx]#[:,[0,2]]

        boundaries = torch.stack([obsamp, ibsamp], dim=0)

        _, boundarycurves = mu.bezierLsqfit(boundaries, 7, M=self.Mboundaryfit)
        boundarycurvesaug = torch.cat([boundarycurves,  torch.ones_like(boundarycurves[:,:,0]).unsqueeze(2)], dim=2).transpose(1,2)
        boundarycurveslocal = torch.matmul(torch.inverse(current_pm.to(self.device)), boundarycurvesaug)[:,0:3].transpose(1,2)[:,:,[0,2]]
        #print(boundarycurveslocal)
        
        boundarypoints = torch.matmul(self.Mboundaryeval, boundarycurveslocal)
        _, boundarytangents = mu.bezierDerivative(boundarycurveslocal, M=self.Mboundarytangenteval)
        boundarytangents = boundarytangents/torch.norm(boundarytangents, dim=2)[:,:,None]
        boundarynormals = boundarytangents.clone().flip(dims=[2])
        boundarynormals[1,:,0]*=-1.0

        obnormals = boundarynormals[0].unsqueeze(0)
        obpoints = boundarypoints[0].unsqueeze(0) - 1.0*obnormals

        
        ibnormals = boundarynormals[1].unsqueeze(0)
        ibpoints = boundarypoints[1].unsqueeze(0) - 1.0*ibnormals

        mask=[True for asdf in range(self.bezier_order+1)]
        mask[0]=not self.fix_first_point
        dT = self.deltaT
        p0 = torch.zeros(2, dtype=self.dtype, device=self.device)
        pf = p0.clone()
        # pf[1]=max(1.75*speed*dT,30.0)
        pf[1]=75.0
        delta = pf - p0
        # initial_guess = torch.stack([p0 + s.item()*delta for s in torch.linspace(0.0,1.0,steps=self.bezier_order+1)], dim=0).unsqueeze(0)
        initial_guess = torch.mean(boundarycurveslocal, dim=0, keepdim=True)
       # print(initial_guess)
        evalpoints = torch.matmul(self.bezierM, initial_guess)

        bcmodel = mu.BezierCurveModule(initial_guess.clone(), mask=mask)
        #  bcmodel.train()
        maxacent = 9.8*3
        maxalinear = 9.8*2
        _, l1 = self.boundary_loss(evalpoints, ibpoints, ibnormals)
        _, l2 = self.boundary_loss(evalpoints, obpoints, obnormals)
        optimizer = SGD(bcmodel.parameters(), lr=0.25, momentum=0.1)
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
            # a_t = a_s/(dT*dT)
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
        bezier_control_points = bcmodel.allControlPoints().detach()
       # print(bezier_control_points)        

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
        