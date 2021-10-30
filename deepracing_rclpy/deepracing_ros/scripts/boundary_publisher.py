import deepracing
import deepracing_msgs.msg
import deepracing_ros
import deepracing_ros.utils

import rclpy
import rclpy.subscription, rclpy.publisher, rclpy.executors
from rclpy.node import Node
from ament_index_python import get_package_share_directory
from ament_index_python import PackageNotFoundError
import sensor_msgs.msg
import std_msgs.msg

import os
import json
import numpy as np
from typing import List

class BoundaryPubNode(Node):
    def __init__(self):
        super(BoundaryPubNode, self).__init__('boundary_pub_node')
        search_dirs_param : rclpy.Parameter = self.declare_parameter("track_search_dirs", value=[])
        self.search_dirs = search_dirs_param.get_parameter_value().string_array_value
        try:
            self.search_dirs.append(os.path.join(get_package_share_directory("f1_datalogger"), "f1_tracks"))
        except PackageNotFoundError as ex:
            pass
        self.current_track_id=-1
        self.current_innerboundary : sensor_msgs.msg.PointCloud2 = None
        self.current_outerboundary : sensor_msgs.msg.PointCloud2 = None
        self.current_racingline : sensor_msgs.msg.PointCloud2 = None

        self.boundary_fields : List[sensor_msgs.msg.PointField] = []
        self.boundary_fields.append(sensor_msgs.msg.PointField(name="x", offset=0, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        self.boundary_fields.append(sensor_msgs.msg.PointField(name="y", offset=4, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        self.boundary_fields.append(sensor_msgs.msg.PointField(name="z", offset=8, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        self.boundary_fields.append(sensor_msgs.msg.PointField(name="r", offset=12, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))

        self.raceline_fields : List[sensor_msgs.msg.PointField] = []
        self.raceline_fields.append(sensor_msgs.msg.PointField(name="x", offset=0, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        self.raceline_fields.append(sensor_msgs.msg.PointField(name="y", offset=4, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        self.raceline_fields.append(sensor_msgs.msg.PointField(name="z", offset=8, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        self.raceline_fields.append(sensor_msgs.msg.PointField(name="time", offset=12, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        self.raceline_fields.append(sensor_msgs.msg.PointField(name="speed", offset=16, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))

        self.inner_boundary_pub : rclpy.publisher.Publisher = self.create_publisher(sensor_msgs.msg.PointCloud2, "inner_boundary", 1)
        self.outer_boundary_pub : rclpy.publisher.Publisher = self.create_publisher(sensor_msgs.msg.PointCloud2, "outer_boundary", 1)
        self.racingline_pub : rclpy.publisher.Publisher = self.create_publisher(sensor_msgs.msg.PointCloud2, "racingline", 1)
        self.session_data_sub : rclpy.subscription.Subscription = self.create_subscription(deepracing_msgs.msg.TimestampedPacketSessionData, "session_data", self.sessionDataCB, 1)

    def sessionDataCB(self, session_data : deepracing_msgs.msg.TimestampedPacketSessionData):
        idx = session_data.udp_packet.track_id
        if not (idx==self.current_track_id):
            now = self.get_clock().now()
            racelinefile = deepracing.searchForFile(deepracing.trackNames[idx] + "_racingline.json", self.search_dirs)
            innerlimitfile = deepracing.searchForFile(deepracing.trackNames[idx] + "_innerlimit.json", self.search_dirs)
            outerlimitfile = deepracing.searchForFile(deepracing.trackNames[idx] + "_outerlimit.json", self.search_dirs)
            with open(racelinefile,"r") as f:
                d : dict = json.load(f)
                raceline : np.ndarray = np.column_stack([d["x"], d["y"], d["z"], d["t"], d["speeds"]])
                raceline = raceline.astype(np.float32)
                raceline_msg : sensor_msgs.msg.PointCloud2 = sensor_msgs.msg.PointCloud2()
                raceline_msg.fields=self.raceline_fields
                raceline_msg.header=std_msgs.msg.Header(frame_id=deepracing_ros.world_coordinate_name, stamp=now.to_msg())
                raceline_msg.is_bigendian=False
                raceline_msg.is_dense=True
                raceline_msg.height=1
                raceline_msg.width=raceline.shape[0]
                raceline_msg.point_step=4*len(raceline_msg.fields)
                raceline_msg.row_step=raceline_msg.point_step*raceline_msg.width
                raceline_msg.data=raceline.flatten().tobytes()
            with open(innerlimitfile,"r") as f:
                d : dict = json.load(f)
                innerboundary : np.ndarray = np.column_stack([d["x"], d["y"], d["z"], d["r"]])
                innerboundary = innerboundary.astype(np.float32)
                innerboundary_msg : sensor_msgs.msg.PointCloud2 = sensor_msgs.msg.PointCloud2()
                innerboundary_msg.fields=self.boundary_fields
                innerboundary_msg.header=std_msgs.msg.Header(frame_id=deepracing_ros.world_coordinate_name, stamp=raceline_msg.header.stamp)
                innerboundary_msg.is_bigendian=False
                innerboundary_msg.is_dense=True
                innerboundary_msg.height=1
                innerboundary_msg.width=innerboundary.shape[0]
                innerboundary_msg.point_step=4*len(innerboundary_msg.fields)
                innerboundary_msg.row_step=innerboundary_msg.point_step*innerboundary_msg.width
                innerboundary_msg.data=innerboundary.flatten().tobytes()
            with open(outerlimitfile,"r") as f:
                d : dict = json.load(f)
                outerboundary : np.ndarray = np.column_stack([d["x"], d["y"], d["z"], d["r"]])
                outerboundary = outerboundary.astype(np.float32)
                outerboundary_msg : sensor_msgs.msg.PointCloud2 = sensor_msgs.msg.PointCloud2()
                outerboundary_msg.fields=self.boundary_fields
                outerboundary_msg.header=std_msgs.msg.Header(frame_id=deepracing_ros.world_coordinate_name, stamp=raceline_msg.header.stamp)
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
            self.current_outerboundary = outerboundary_msg

    def timerCB(self):
        if self.current_racingline is not None:
            self.racingline_pub.publish(self.current_racingline)
        if self.current_innerboundary is not None:
            self.inner_boundary_pub.publish(self.current_innerboundary)
        if self.current_outerboundary is not None:
            self.outer_boundary_pub.publish(self.current_outerboundary)
    


def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = BoundaryPubNode()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
    executor : rclpy.executors.MultiThreadedExecutor = rclpy.executors.MultiThreadedExecutor(3)
    executor.add_node(node)
    node.create_timer(1.0/5.0, node.timerCB)
    executor.spin()