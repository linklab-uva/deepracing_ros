from requests import head
import deepracing
import deepracing_msgs.msg
import deepracing_ros
import deepracing_ros.utils

import rclpy
import rclpy.subscription, rclpy.publisher, rclpy.executors
from rclpy.node import Node
from ament_index_python import get_package_share_directory
from ament_index_python import PackageNotFoundError
import geometry_msgs.msg
from rclpy.time import Time
from rclpy.duration import Duration
import sensor_msgs.msg
import nav_msgs.msg
import std_msgs.msg
import tf2_ros
import sensor_msgs_py.point_cloud2



import math
import os
import json
import numpy as np
from typing import List
from scipy.spatial.transform import Rotation
import scipy.interpolate
import rcl_interfaces.msg

class BoundaryPubNode(Node):
    def __init__(self):
        super(BoundaryPubNode, self).__init__('boundary_pub_node')
        search_dirs_descriptor : rcl_interfaces.msg.ParameterDescriptor = rcl_interfaces.msg.ParameterDescriptor()
        search_dirs_descriptor.name="track_search_dirs"
        search_dirs_descriptor.type=rcl_interfaces.msg.ParameterType.PARAMETER_STRING_ARRAY
        search_dirs_param : rclpy.Parameter = self.declare_parameter(search_dirs_descriptor.name, descriptor=search_dirs_descriptor)
        self.search_dirs = search_dirs_param.get_parameter_value().string_array_value
        try:
            self.search_dirs.append(os.path.join(get_package_share_directory("f1_datalogger"), "f1_tracks", "minimumcurvature"))
            self.search_dirs.append(os.path.join(get_package_share_directory("f1_datalogger"), "f1_tracks"))
        except PackageNotFoundError as ex:
            pass
        searchdirs_string : str = ""
        for searchdir in self.search_dirs:
            searchdirs_string = searchdirs_string+searchdir+","
        self.get_logger().info("Looking for track files in %s." % (searchdirs_string[0:-1]))
        self.current_track_id=-1
        self.current_innerboundary : nav_msgs.msg.Path = None
        self.current_outerboundary : nav_msgs.msg.Path = None
        self.current_innerboundary_pc2 : sensor_msgs.msg.PointCloud2 = None
        self.current_outerboundary_pc2 : sensor_msgs.msg.PointCloud2 = None
        self.pc2_fields = []
        self.pc2_fields.append(sensor_msgs.msg.PointField(name='x', offset=0, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        self.pc2_fields.append(sensor_msgs.msg.PointField(name='y', offset=4, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        self.pc2_fields.append(sensor_msgs.msg.PointField(name='z', offset=8, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        self.pc2_fields.append(sensor_msgs.msg.PointField(name='nx', offset=12, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        self.pc2_fields.append(sensor_msgs.msg.PointField(name='ny', offset=16, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        self.pc2_fields.append(sensor_msgs.msg.PointField(name='nz', offset=20, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))

        self.inner_boundary_pub : rclpy.publisher.Publisher = self.create_publisher(nav_msgs.msg.Path, "/inner_boundary", 1)
        self.outer_boundary_pub : rclpy.publisher.Publisher = self.create_publisher(nav_msgs.msg.Path, "/outer_boundary", 1)
        self.inner_boundary_pc2_pub : rclpy.publisher.Publisher = self.create_publisher(sensor_msgs.msg.PointCloud2, "/inner_boundary/pc2", 1)
        self.outer_boundary_pc2_pub : rclpy.publisher.Publisher = self.create_publisher(sensor_msgs.msg.PointCloud2, "/outer_boundary/pc2", 1)
        self.session_data_sub : rclpy.subscription.Subscription = self.create_subscription(deepracing_msgs.msg.TimestampedPacketSessionData, "session_data", self.sessionDataCB, 1)

        self.tf2_buffer : tf2_ros.Buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5))
        self.tf2_listener : tf2_ros.TransformListener = tf2_ros.TransformListener(self.tf2_buffer, self, spin_thread=False)
              



    def sessionDataCB(self, session_data : deepracing_msgs.msg.TimestampedPacketSessionData):
        # self.get_logger().info("Got some session data")
        idx = session_data.udp_packet.track_id
        if not (idx==self.current_track_id):
            innerlimitfile = deepracing.searchForFile(deepracing.trackNames[idx] + "_innerlimit.json", self.search_dirs)
            outerlimitfile = deepracing.searchForFile(deepracing.trackNames[idx] + "_outerlimit.json", self.search_dirs)
            transform_msg : geometry_msgs.msg.TransformStamped = self.tf2_buffer.lookup_transform("map", deepracing_ros.world_coordinate_name, Time())
            transform_vec : geometry_msgs.msg.Vector3 = transform_msg.transform.translation
            transform_quat : geometry_msgs.msg.Quaternion = transform_msg.transform.rotation
            Tmat : np.ndarray = np.eye(4, dtype=np.float64)
            Tmat[0:3,0:3] = Rotation.from_quat([transform_quat.x, transform_quat.y, transform_quat.z, transform_quat.w]).as_matrix().astype(Tmat.dtype)
            Tmat[0:3,3] = np.asarray([transform_vec.x, transform_vec.y, transform_vec.z], dtype=Tmat.dtype)
            Rmat = Tmat[0:3,0:3].copy()
            with open(innerlimitfile,"r") as f:
                d : dict = json.load(f)
            innerboundary_points : np.ndarray = np.matmul(  Tmat, np.row_stack([ d["x"],  d["y"],  d["z"], np.ones_like(d["z"]) ]).astype(Tmat.dtype) )[0:3].transpose()
            # innerboundary_points[:,2] = np.mean(innerboundary_points[:,2])
            innerboundary_r : np.ndarray = np.asarray(d["r"], dtype=Tmat.dtype)
            innerboundary_spline : scipy.interpolate.BSpline = scipy.interpolate.make_interp_spline(innerboundary_r, innerboundary_points)
            innerboundary_tangent_spline : scipy.interpolate.BSpline = innerboundary_spline.derivative()
            innerboundary_xvecs : np.ndarray = innerboundary_tangent_spline(innerboundary_r)
            innerboundary_xvecs = innerboundary_xvecs/(np.linalg.norm(innerboundary_xvecs, ord=2, axis=1)[:,np.newaxis])

            down : np.ndarray = np.zeros_like(innerboundary_xvecs)
            down[:,2]=-1.0

            innerboundary_yvecs : np.ndarray = np.cross(down, innerboundary_xvecs)
            innerboundary_yvecs = innerboundary_yvecs/(np.linalg.norm(innerboundary_yvecs, ord=2, axis=1)[:,np.newaxis])

            innerboundary_zvecs : np.ndarray = np.cross(innerboundary_xvecs, innerboundary_yvecs)
            innerboundary_zvecs = innerboundary_zvecs/(np.linalg.norm(innerboundary_zvecs, ord=2, axis=1)[:,np.newaxis])

            innerboundary_rotmats : np.ndarray = np.zeros((innerboundary_xvecs.shape[0], 3, 3), dtype=innerboundary_xvecs.dtype)
            innerboundary_rotmats[:,:,0] = innerboundary_xvecs
            innerboundary_rotmats[:,:,1] = innerboundary_yvecs
            innerboundary_rotmats[:,:,2] = innerboundary_zvecs

            innerboundary_arr : np.ndarray = np.concatenate([innerboundary_points, innerboundary_yvecs], axis=1).astype(np.float32)

            innerboundary_rotations : Rotation = Rotation.from_matrix(innerboundary_rotmats)
            innerboundary_quaternions : np.ndarray = innerboundary_rotations.as_quat()

            innerboundary_msg : nav_msgs.msg.Path = nav_msgs.msg.Path()
            innerboundary_msg.header.frame_id=transform_msg.header.frame_id
            for i in range(innerboundary_quaternions.shape[0]):
                point : np.ndarray = innerboundary_points[i]
                quat : np.ndarray = innerboundary_quaternions[i]
                pose : geometry_msgs.msg.PoseStamped = geometry_msgs.msg.PoseStamped(header=innerboundary_msg.header)
                d, i  = math.modf(innerboundary_r[i])
                pose.header.stamp.sec=int(round(i))
                pose.header.stamp.nanosec=int(round(d*1E9))
                pose.pose.position.x=point[0]
                pose.pose.position.y=point[1]
                pose.pose.position.z=point[2]
                pose.pose.orientation.x=quat[0]
                pose.pose.orientation.y=quat[1]
                pose.pose.orientation.z=quat[2]
                pose.pose.orientation.w=quat[3]
                innerboundary_msg.poses.append(pose)
            with open(outerlimitfile,"r") as f:
                d : dict = json.load(f)
            outerboundary_points : np.ndarray = np.matmul(  Tmat, np.row_stack([ d["x"],  d["y"],  d["z"], np.ones_like(d["z"]) ]).astype(Tmat.dtype) )[0:3].transpose()
            # outerboundary_points[:,2] = np.mean(outerboundary_points[:,2])
            outerboundary_r : np.ndarray = np.asarray(d["r"], dtype=Tmat.dtype)
            outerboundary_spline : scipy.interpolate.BSpline = scipy.interpolate.make_interp_spline(outerboundary_r, outerboundary_points)
            outerboundary_tangent_spline : scipy.interpolate.BSpline = outerboundary_spline.derivative()
            outerboundary_xvecs : np.ndarray = outerboundary_tangent_spline(outerboundary_r)
            outerboundary_xvecs = outerboundary_xvecs/(np.linalg.norm(outerboundary_xvecs, ord=2, axis=1)[:,np.newaxis])

            up : np.ndarray = np.zeros_like(outerboundary_xvecs)
            up[:,2]=1.0

            outerboundary_yvecs : np.ndarray = np.cross(up, outerboundary_xvecs)
            outerboundary_yvecs = outerboundary_yvecs/(np.linalg.norm(outerboundary_yvecs, ord=2, axis=1)[:,np.newaxis])

            outerboundary_zvecs : np.ndarray = np.cross(outerboundary_xvecs, outerboundary_yvecs)
            outerboundary_zvecs = outerboundary_zvecs/(np.linalg.norm(outerboundary_zvecs, ord=2, axis=1)[:,np.newaxis])

            outerboundary_rotmats : np.ndarray = np.zeros((outerboundary_xvecs.shape[0], 3, 3), dtype=outerboundary_xvecs.dtype)
            outerboundary_rotmats[:,:,0] = outerboundary_xvecs
            outerboundary_rotmats[:,:,1] = outerboundary_yvecs
            outerboundary_rotmats[:,:,2] = outerboundary_zvecs

            outerboundary_rotations : Rotation = Rotation.from_matrix(outerboundary_rotmats)
            outerboundary_quaternions : np.ndarray = outerboundary_rotations.as_quat()

            outerboundary_arr : np.ndarray = np.concatenate([outerboundary_points, outerboundary_yvecs], axis=1).astype(np.float32)

            outerboundary_msg : nav_msgs.msg.Path = nav_msgs.msg.Path()
            outerboundary_msg.header.frame_id=transform_msg.header.frame_id
            for i in range(outerboundary_quaternions.shape[0]):
                point : np.ndarray = outerboundary_points[i]
                quat : np.ndarray = outerboundary_quaternions[i]
                pose : geometry_msgs.msg.PoseStamped = geometry_msgs.msg.PoseStamped(header=outerboundary_msg.header)
                d, i = math.modf(outerboundary_r[i])
                pose.header.stamp.sec=int(round(i))
                pose.header.stamp.nanosec=int(round(d*1E9))
                pose.pose.position.x=point[0]
                pose.pose.position.y=point[1]
                pose.pose.position.z=point[2]
                pose.pose.orientation.x=quat[0]
                pose.pose.orientation.y=quat[1]
                pose.pose.orientation.z=quat[2]
                pose.pose.orientation.w=quat[3]
                outerboundary_msg.poses.append(pose)


            self.current_track_id = idx
            self.current_innerboundary = innerboundary_msg
            self.current_outerboundary = outerboundary_msg
            self.current_innerboundary_pc2 = sensor_msgs_py.point_cloud2.create_cloud(innerboundary_msg.header, self.pc2_fields, innerboundary_arr.tolist())
            self.current_outerboundary_pc2 = sensor_msgs_py.point_cloud2.create_cloud(outerboundary_msg.header, self.pc2_fields, outerboundary_arr.tolist())

    def timerCB(self):
        now = self.get_clock().now()
        if self.current_innerboundary is not None:
            self.current_innerboundary.header.stamp=now.to_msg()
            self.inner_boundary_pub.publish(self.current_innerboundary)
        if self.current_outerboundary is not None:
            self.current_outerboundary.header.stamp=now.to_msg()
            self.outer_boundary_pub.publish(self.current_outerboundary)
        if self.current_innerboundary_pc2 is not None:
            self.current_innerboundary_pc2.header.stamp=now.to_msg()
            self.inner_boundary_pc2_pub.publish(self.current_innerboundary_pc2)
        if self.current_outerboundary_pc2 is not None:
            self.current_outerboundary_pc2.header.stamp=now.to_msg()
            self.outer_boundary_pc2_pub.publish(self.current_outerboundary_pc2)
    


def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = BoundaryPubNode()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
    executor : rclpy.executors.MultiThreadedExecutor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    node.create_timer(2.0, node.timerCB)
    executor.spin()