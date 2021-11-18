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
import std_msgs.msg
import tf2_ros



import os
import json
import numpy as np
from typing import List
from scipy.spatial.transform import Rotation

class BoundaryPubNode(Node):
    def __init__(self):
        super(BoundaryPubNode, self).__init__('boundary_pub_node')
        search_dirs_param : rclpy.Parameter = self.declare_parameter("track_search_dirs", value=[])
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
        self.current_innerboundary : sensor_msgs.msg.PointCloud2 = None
        self.current_outerboundary : sensor_msgs.msg.PointCloud2 = None
        self.current_racingline : sensor_msgs.msg.PointCloud2 = None

        self.boundary_fields : List[sensor_msgs.msg.PointField] = []
        self.boundary_fields.append(sensor_msgs.msg.PointField(name="x", offset=0, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        self.boundary_fields.append(sensor_msgs.msg.PointField(name="y", offset=4, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        self.boundary_fields.append(sensor_msgs.msg.PointField(name="z", offset=8, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        self.boundary_fields.append(sensor_msgs.msg.PointField(name="nx", offset=12, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        self.boundary_fields.append(sensor_msgs.msg.PointField(name="ny", offset=16, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        self.boundary_fields.append(sensor_msgs.msg.PointField(name="nz", offset=20, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        self.boundary_fields.append(sensor_msgs.msg.PointField(name="r", offset=24, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))

        self.raceline_fields : List[sensor_msgs.msg.PointField] = []
        self.raceline_fields.append(sensor_msgs.msg.PointField(name="x", offset=0, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        self.raceline_fields.append(sensor_msgs.msg.PointField(name="y", offset=4, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        self.raceline_fields.append(sensor_msgs.msg.PointField(name="z", offset=8, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        self.raceline_fields.append(sensor_msgs.msg.PointField(name="time", offset=12, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        self.raceline_fields.append(sensor_msgs.msg.PointField(name="speed", offset=16, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))

        self.inner_boundary_pub : rclpy.publisher.Publisher = self.create_publisher(sensor_msgs.msg.PointCloud2, "inner_boundary", 1)
        self.outer_boundary_pub : rclpy.publisher.Publisher = self.create_publisher(sensor_msgs.msg.PointCloud2, "outer_boundary", 1)
        self.racingline_pub : rclpy.publisher.Publisher = self.create_publisher(sensor_msgs.msg.PointCloud2, "optimal_raceline", 1)
        self.session_data_sub : rclpy.subscription.Subscription = self.create_subscription(deepracing_msgs.msg.TimestampedPacketSessionData, "session_data", self.sessionDataCB, 1)

        self.tf2_buffer : tf2_ros.Buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5))
        self.tf2_listener : tf2_ros.TransformListener = tf2_ros.TransformListener(self.tf2_buffer, self, spin_thread=False)



    def sessionDataCB(self, session_data : deepracing_msgs.msg.TimestampedPacketSessionData):
        idx = session_data.udp_packet.track_id
        if not (idx==self.current_track_id):
            now = self.get_clock().now()
            racelinefile = deepracing.searchForFile(deepracing.trackNames[idx] + "_minimumcurvaturelowca.json", self.search_dirs)
            innerlimitfile = deepracing.searchForFile(deepracing.trackNames[idx] + "_innerlimit.json", self.search_dirs)
            outerlimitfile = deepracing.searchForFile(deepracing.trackNames[idx] + "_outerlimit.json", self.search_dirs)
            transform_msg : geometry_msgs.msg.TransformStamped = self.tf2_buffer.lookup_transform("map", deepracing_ros.world_coordinate_name, Time())
            transform_vec : geometry_msgs.msg.Vector3 = transform_msg.transform.translation
            transform_quat : geometry_msgs.msg.Quaternion = transform_msg.transform.rotation
            Tmat : np.ndarray = np.eye(4, dtype=np.float32)
            Tmat[0:3,0:3] = Rotation.from_quat([transform_quat.x, transform_quat.y, transform_quat.z, transform_quat.w]).as_matrix().astype(np.float32)
            Tmat[0:3,3] = np.asarray([transform_vec.x, transform_vec.y, transform_vec.z], dtype=np.float32)
            Rmat = Tmat[0:3,0:3].copy()
            with open(racelinefile,"r") as f:
                d : dict = json.load(f)
                raceline : np.ndarray = np.row_stack([d["x"], d["y"], d["z"], np.ones_like(d["z"]), d["speeds"]])
                raceline = raceline.astype(np.float32)
                raceline[0:3]=np.matmul(Tmat[0:3], raceline[0:4])
                raceline[3]=np.asarray(d["t"], dtype=np.float32)
                raceline_msg : sensor_msgs.msg.PointCloud2 = sensor_msgs.msg.PointCloud2()
                raceline_msg.fields=self.raceline_fields
                raceline_msg.header=std_msgs.msg.Header(frame_id=transform_msg.header.frame_id, stamp=now.to_msg())
                raceline_msg.is_bigendian=False
                raceline_msg.is_dense=True
                raceline_msg.height=1
                raceline_msg.width=raceline.shape[1]
                raceline_msg.point_step=4*len(raceline_msg.fields)
                raceline_msg.row_step=raceline_msg.point_step*raceline_msg.width
                raceline_msg.data=raceline.transpose().flatten().tobytes()
            with open(innerlimitfile,"r") as f:
                d : dict = json.load(f)
                innerboundarypoints : np.ndarray = np.matmul(  Tmat, np.row_stack([ d["x"],  d["y"],  d["z"], np.ones_like(d["z"]) ]).astype(np.float32) )[0:3].transpose()
                innerboundarynormals : np.ndarray = np.matmul( Rmat, np.row_stack([ d["nx"], d["ny"], d["nz"] ]).astype(np.float32) ).transpose()
                innerboundary_r : np.ndarray = np.asarray(d["r"], dtype=np.float32)
                innerboundary : np.ndarray = np.column_stack([innerboundarypoints, innerboundarynormals, innerboundary_r])

                innerboundary_msg : sensor_msgs.msg.PointCloud2 = sensor_msgs.msg.PointCloud2()
                innerboundary_msg.fields=self.boundary_fields
                innerboundary_msg.header=raceline_msg.header
                innerboundary_msg.is_bigendian=False
                innerboundary_msg.is_dense=True
                innerboundary_msg.height=1
                innerboundary_msg.width=innerboundary.shape[0]
                innerboundary_msg.point_step=4*len(innerboundary_msg.fields)
                innerboundary_msg.row_step=innerboundary_msg.point_step*innerboundary_msg.width
                innerboundary_msg.data=innerboundary.flatten().tobytes()
            with open(outerlimitfile,"r") as f:
                d : dict = json.load(f)
                outerboundarypoints : np.ndarray = np.matmul(  Tmat, np.row_stack([ d["x"],  d["y"],  d["z"], np.ones_like(d["z"]) ]).astype(np.float32))[0:3].transpose()
                outerboundarynormals : np.ndarray = np.matmul( Rmat, np.row_stack([ d["nx"], d["ny"], d["nz"] ]).astype(np.float32) ).transpose()
                outerboundary_r : np.ndarray = np.asarray(d["r"], dtype=np.float32)
                outerboundary : np.ndarray = np.column_stack([outerboundarypoints, outerboundarynormals, outerboundary_r])

                outerboundary_msg : sensor_msgs.msg.PointCloud2 = sensor_msgs.msg.PointCloud2()
                outerboundary_msg.fields=self.boundary_fields
                outerboundary_msg.header=raceline_msg.header
                outerboundary_msg.is_bigendian=False
                outerboundary_msg.is_dense=True
                outerboundary_msg.height=1
                outerboundary_msg.width=outerboundary.shape[0]
                outerboundary_msg.point_step=4*len(outerboundary_msg.fields)
                outerboundary_msg.row_step=outerboundary_msg.point_step*outerboundary_msg.width
                outerboundary_msg.data=outerboundary.flatten().tobytes()

            self.current_track_id = idx
            self.current_racingline = raceline_msg
            self.current_innerboundary = innerboundary_msg
            self.current_outerboundary= outerboundary_msg

    def timerCB(self):
        now = self.get_clock().now()
        if self.current_racingline is not None:
            self.current_racingline.header.stamp=now.to_msg()
            self.racingline_pub.publish(self.current_racingline)
        if self.current_innerboundary is not None:
            self.current_innerboundary.header.stamp=now.to_msg()
            self.inner_boundary_pub.publish(self.current_innerboundary)
        if self.current_outerboundary is not None:
            self.current_outerboundary.header.stamp=now.to_msg()
            self.outer_boundary_pub.publish(self.current_outerboundary)
    


def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = BoundaryPubNode()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
    executor : rclpy.executors.MultiThreadedExecutor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    node.create_timer(1.0/120.0, node.timerCB)
    executor.spin()