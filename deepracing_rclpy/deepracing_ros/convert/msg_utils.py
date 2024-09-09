from torch.functional import Tensor
import deepracing_msgs.msg as drmsgs # BezierCurve, TimestampedPacketMotionData, PacketMotionData, CarMotionData, PacketHeader
import geometry_msgs.msg as geo_msgs#  Point, PointStamped, Vector3, Vector3Stamped
import nav_msgs.msg as nav_msgs
import tf2_msgs.msg as tf2_msgs
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from builtin_interfaces.msg import Time, Duration
import numpy as np
import numpy.linalg as la
import scipy.spatial.transform
import math
import struct
from typing import List, Union, Tuple
from scipy.spatial.transform import Rotation as Rot
import deepracing
import torch
import sys

# _DATATYPES = {
# PointField.INT8    : ('b', 1),\
# PointField.UINT8  : ('B', 1),\
# PointField.INT16   : ('h', 2),\
# PointField.UINT16 : ('H', 2),\
# PointField.INT32  : ('i', 4),\
# PointField.UINT3  : ('I', 4),\
# PointField.FLOAT32 : ('f', 4),\
# PointField.FLOAT64 : ('d', 8)
# }
_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)



def arrayToPointCloud2(pointsarray : Union[torch.Tensor, np.ndarray], field_names : List[str], header : Header, is_bigendian = False):
   if isinstance(pointsarray, torch.Tensor):
      points = pointsarray.detach().cpu().numpy()
   elif isinstance(pointsarray, np.ndarray):
      points = pointsarray
   else:
      raise TypeError("arrayToPointCloud2 only supports torch.Tensor and np.ndarray as input. Got unknown type: %s" %(str(type(pointsarray)),))

   numfields = len(field_names)
   assert(numfields==points.shape[1])
   pc2out = PointCloud2(header=header, is_bigendian = is_bigendian, is_dense = True, width = points.shape[0], height = 1)
   if points.dtype==np.float64:
      bytesperfield = 8
      dtypemsg = PointField.FLOAT64
   elif points.dtype==np.float32:
      bytesperfield = 4
      dtypemsg = PointField.FLOAT32
   else:
      raise ValueError("Only float32 and float64 arrays are supported")
   pc2out.point_step = bytesperfield*numfields
   pc2out.row_step=pc2out.point_step*pc2out.width
   pc2out.fields=[PointField(name=name, offset=i*bytesperfield, count=1, datatype=dtypemsg) for (i,name) in enumerate(field_names)]
   pc2out.data = points.flatten().tobytes()
   return pc2out

def extractAcceleration(packet : drmsgs.PacketMotionData , car_index : int | str = "player") -> np.ndarray:
   if car_index == "player":
      idx = packet.header.player_car_index
      if idx<0 or idx>22:
         raise ValueError("cannot extract player car index: %d" % (idx,))
   elif car_index == "secondary":
      idx = packet.header.secondary_player_car_index
      if idx<0 or idx>22:
         raise ValueError("cannot extract secondary player car index: %d" % (idx,))
   else:
      idx = car_index
      if idx<0 or idx>22:
         raise ValueError("cannot extract provided index: %d" % (idx,))
   motion_data : drmsgs.CarMotionData = packet.car_motion_data[idx]
   acceleration = 9.8*np.array( (motion_data.g_force_longitudinal, motion_data.g_force_lateral, motion_data.g_force_vertical), dtype=np.float64)
   return acceleration 

def extractAngularVelocity(packet : drmsgs.PacketMotionExData) -> np.ndarray:
   angular_velocity = np.array( (packet.angular_velocity.x, packet.angular_velocity.y, packet.angular_velocity.z), dtype=np.float64)
   return angular_velocity 

def extractVelocity(packet : drmsgs.PacketMotionData , car_index = -1) -> np.ndarray:
   if car_index ==-1:
      idx = packet.header.player_car_index
      if idx<0 or idx>22:
         raise ValueError("cannot extract player car index: %d" % (idx,))
   elif car_index ==-2:
      idx = packet.header.secondary_player_car_index
      if idx<0 or idx>22:
         raise ValueError("cannot extract secondary player car index: %d" % (idx,))
   else:
      idx = car_index
      if idx<0 or idx>22:
         raise ValueError("cannot extract provided index: %d" % (idx,))
   motion_data : drmsgs.CarMotionData = packet.car_motion_data[idx]
   velocity = np.array( (motion_data.world_velocity.vector.x, motion_data.world_velocity.vector.y, motion_data.world_velocity.vector.z), dtype=np.float64)
   return velocity 

def extractPosition(packet : drmsgs.PacketMotionData , car_index = -1) -> np.ndarray:
   if car_index ==-1:
      idx = packet.header.player_car_index
      if idx<0 or idx>22:
         raise ValueError("cannot extract player car index: %d" % (idx,))
   elif car_index ==-2:
      idx = packet.header.secondary_player_car_index
      if idx<0 or idx>22:
         raise ValueError("cannot extract secondary player car index: %d" % (idx,))
   else:
      idx = car_index
      if idx<0 or idx>22:
         raise ValueError("cannot extract provided index: %d" % (idx,))
   motion_data : drmsgs.CarMotionData = packet.car_motion_data[idx]
   position = np.array( (motion_data.world_position.point.x, motion_data.world_position.point.y, motion_data.world_position.point.z), dtype=np.float64)
   return position 

def extractOrientation(packet : drmsgs.PacketMotionData, car_index = -1) -> scipy.spatial.transform.Rotation:
   if car_index ==-1:
      idx = packet.header.player_car_index
      if idx<0 or idx>22:
         raise ValueError("cannot extract player car index: %d" % (idx,))
   elif car_index ==-2:
      idx = packet.header.secondary_player_car_index
      if idx<0 or idx>22:
         raise ValueError("cannot extract secondary player car index: %d" % (idx,))
   else:
      idx = car_index
      if idx<0 or idx>22:
         raise ValueError("cannot extract provided index: %d" % (idx,))
   motion_data : drmsgs.CarMotionData = packet.car_motion_data[idx]
   try:
      leftdir : geo_msgs.Vector3 = motion_data.world_left_dir.vector
      leftvector = np.array((leftdir.x, leftdir.y, leftdir.z), dtype=np.float64)
   except AttributeError as e:
      rightdir : geo_msgs.Vector3 = motion_data.world_right_dir.vector
      leftvector = -np.array((rightdir.x, rightdir.y, rightdir.z), dtype=np.float64)

   forwarddir : geo_msgs.Vector3 = motion_data.world_forward_dir.vector

   leftvector = leftvector/la.norm(leftvector)

   forwardvector = np.array((forwarddir.x, forwarddir.y, forwarddir.z), dtype=np.float64)
   forwardvector = forwardvector/la.norm(forwardvector)

   upvector = np.cross(forwardvector,leftvector)
   upvector = upvector/la.norm(upvector)
   rotationmat = np.column_stack([forwardvector,leftvector,upvector])
   return scipy.spatial.transform.Rotation.from_matrix(rotationmat)

def extractPose(packet : drmsgs.PacketMotionData, car_index = -1) -> Tuple[np.ndarray, scipy.spatial.transform.Rotation]:
   if car_index ==-1:
      idx = packet.header.player_car_index
      if idx<0 or idx>22:
         raise ValueError("cannot extract player car index: %d" % (idx,))
   elif car_index ==-2:
      idx = packet.header.secondary_player_car_index
      if idx<0 or idx>22:
         raise ValueError("cannot extract secondary player car index: %d" % (idx,))
   else:
      idx = car_index
      if idx<0 or idx>22:
         raise ValueError("cannot extract provided index: %d" % (idx,))
   return extractPosition(packet, car_index=idx).astype(np.float64), extractOrientation(packet, car_index=idx)

def toBezierCurveMsg(control_points : torch.Tensor, header: Header, covars : Union[torch.Tensor, None] = None, delta_t : Duration = Duration()):
   ptsnp : np.ndarray = control_points.detach().cpu().numpy()
   if covars is not None:
      assert(ptsnp.shape[0]==covars.shape[0])
      covarmatsnp : np.ndarray = covars.view(covars.shape[0],9).detach().cpu().numpy()
   rtn : drmsgs.BezierCurve = drmsgs.BezierCurve(header=header, delta_t=delta_t)
   for i in range(ptsnp.shape[0]):
      rtn.control_points.append(geo_msgs.Point(x=float(ptsnp[i,0]), y=float(ptsnp[i,1]), z=float(ptsnp[i,2])))
      if covars is not None: 
         rtn.control_point_covariances.append(drmsgs.CovariancePoint(covariance = covarmatsnp[i]))
   return rtn
def fromBezierCurveMsg(curve_msg : drmsgs.BezierCurve, dtype=torch.float32, device=torch.device("cpu")):
   ptsnp = np.array([[p.x, p.y, p.z ] for p in curve_msg.control_points ], copy=False)
   covarslength = len(curve_msg.control_point_covariances)
   if covarslength>0:
      if not (covarslength==ptsnp.shape[0]):
         raise ValueError("Tried to unpack a bezier curve with %d control points, but with %d covariance matrices. A BezierCurve message must either have 0 covariances or the the same as the number of control points." % (ptsnp.shape[0], covarslength))
      covariances = torch.stack([torch.as_tensor(curve_msg.control_point_covariances[i].covariance, dtype=dtype, device=device) for i in range(covarslength)], dim=0).view(covarslength,3,3)
   else:
      covariances = None
   return torch.as_tensor(ptsnp.copy(), device=device, dtype=dtype), covariances
   
def transformMsgToTorch(transform_msg: geo_msgs.Transform, dtype=torch.float32, device=torch.device("cpu"), requires_grad=False):
   rtn = torch.eye(4, dtype=dtype, device=device, requires_grad=requires_grad)
   rtn[0:3,0:3] = torch.as_tensor(Rot.from_quat(np.array([transform_msg.rotation.x, transform_msg.rotation.y, transform_msg.rotation.z, transform_msg.rotation.w], copy=False)).as_matrix().copy(), dtype=dtype, device=device)
   rtn[0:3,3] = torch.as_tensor(np.array([transform_msg.translation.x, transform_msg.translation.y, transform_msg.translation.z]), dtype=dtype, device=device)
   return rtn

def poseMsgToTorch(pose_msg: geo_msgs.Pose, dtype=torch.float32, device=torch.device("cpu"), requires_grad=False) ->  torch.Tensor:
   rtn = torch.eye(4, dtype=dtype, device=device, requires_grad=requires_grad)
   r : Rot = Rot.from_quat([pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w])
   rtn[0:3,0:3] = torch.as_tensor(r.as_matrix(), dtype=dtype, device=device).requires_grad_(requires_grad)
   rtn[0:3,3] = torch.as_tensor([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z], dtype=dtype, device=device).requires_grad_(requires_grad)
   return rtn

def pointMsgToTorch(point_msg: geo_msgs.Point, dtype=torch.float32, device=torch.device("cpu")):
   return torch.as_tensor([point_msg.x, point_msg.y, point_msg.z], dtype=dtype, device=device)

def vectorMsgToTorch(vector_msg: geo_msgs.Vector3, dtype=torch.float32, device=torch.device("cpu")):
   return torch.as_tensor([vector_msg.x, vector_msg.y, vector_msg.z], dtype=dtype, device=device)

def torchToVector3Msg(vector_torch : torch.Tensor) -> geo_msgs.Vector3:
   return geo_msgs.Vector3(x = vector_torch[0].item(), y = vector_torch[1].item(), z = vector_torch[2].item())

def torchToPointMsg(point_torch : torch.Tensor) -> geo_msgs.Point:
   return geo_msgs.Point(x = point_torch[0].item(), y = point_torch[1].item(), z = point_torch[2].item())

def torchToPoseMsg(pose_torch : torch.Tensor) -> geo_msgs.Pose:
   rtn : geo_msgs.Pose =  geo_msgs.Pose(position = torchToPointMsg(pose_torch[0:3,3]))
   rotsp : Rot = Rot.from_matrix(pose_torch[0:3,0:3].cpu().numpy())
   quatnp = rotsp.as_quat()
   rtn.orientation = geo_msgs.Quaternion(x = float(quatnp[0]), y = float(quatnp[1]), z = float(quatnp[2]), w = float(quatnp[3]))
   return rtn

def numpifySessionData(session_data_msgs : list[drmsgs.TimestampedPacketSessionData], float_type=np.float32, int_type=np.int32) -> \
   dict[str, np.ndarray]:
   numpackets = len(session_data_msgs)
   if numpackets<=0:
      raise ValueError("input list cannot be empty")
   session_times = np.empty(numpackets, dtype=float_type)
   frame_identifiers = np.empty(numpackets, dtype=int_type)
   overall_frame_identifiers = np.empty(numpackets, dtype=int_type)
   game_modes = np.empty(numpackets, dtype=int_type)
   game_paused = np.empty(numpackets, dtype=int_type)
   session_types = np.empty(numpackets, dtype=int_type)
   track_ids = np.empty(numpackets, dtype=int_type)
   for packetnumber in range(numpackets):
      session_data : drmsgs.PacketSessionData = session_data_msgs[packetnumber].udp_packet
      session_times[packetnumber] = session_data.header.session_time
      frame_identifiers[packetnumber] = session_data.header.frame_identifier
      try:
         overall_frame_identifiers[packetnumber] = session_data.header.overall_frame_identifier
      except AttributeError as e:
         overall_frame_identifiers[packetnumber] = session_data.header.frame_identifier
      track_ids[packetnumber] = session_data.track_id
      game_modes[packetnumber] = session_data.game_mode
      game_paused[packetnumber] = session_data.game_paused
      session_types[packetnumber] = session_data.session_type
   return { 
      "session_times": session_times, 
      "frame_identifiers": frame_identifiers, 
      "overall_frame_identifiers": overall_frame_identifiers, 
      "track_ids": track_ids, 
      "game_modes": game_modes, 
      "session_types": session_types, 
      "game_paused": game_paused 
   }
def numpifyTelemetryData(telemetry_data_msgs : list[drmsgs.TimestampedPacketCarTelemetryData], float_type=np.float32, int_type=np.int32) -> \
   dict[str, np.ndarray]:
   numpackets = len(telemetry_data_msgs)
   if numpackets<=0:
      raise ValueError("input list cannot be empty")
   numdrivers = len(telemetry_data_msgs[0].udp_packet.car_telemetry_data)
   session_times = np.empty(numpackets, dtype=float_type)
   frame_identifiers = np.empty(numpackets, dtype=int_type)
   overall_frame_identifiers = np.empty(numpackets, dtype=int_type)
   brake = np.empty([numdrivers, numpackets], dtype=float_type)
   throttle = np.empty([numdrivers, numpackets], dtype=float_type)
   steering = np.empty([numdrivers, numpackets], dtype=float_type)
   speed = np.empty([numdrivers, numpackets], dtype=int_type)
   gear = np.empty([numdrivers, numpackets], dtype=int_type)
   rev_lights_percent = np.empty([numdrivers, numpackets], dtype=int_type)
   surface_type = np.empty([numdrivers, numpackets, 4], dtype=np.uint8)
   tire_inner_temp = np.empty([numdrivers, numpackets, 4], dtype=np.uint8)
   tire_surface_temp = np.empty([numdrivers, numpackets, 4], dtype=np.uint8)
   for packetnumber in range(numpackets):
      session_times[packetnumber] = telemetry_data_msgs[packetnumber].udp_packet.header.session_time
      telemetry_packet = telemetry_data_msgs[packetnumber].udp_packet
      frame_identifiers[packetnumber] = telemetry_packet.header.frame_identifier
      try:
         overall_frame_identifiers[packetnumber] = telemetry_packet.header.overall_frame_identifier
      except AttributeError as e:
         overall_frame_identifiers[packetnumber] = telemetry_packet.header.frame_identifier
      for carindex in range(numdrivers):
         current_telemetry : drmsgs.CarTelemetryData = telemetry_packet.car_telemetry_data[carindex]
         throttle[carindex, packetnumber] = current_telemetry.throttle
         brake[carindex, packetnumber] = current_telemetry.brake
         speed[carindex, packetnumber] = current_telemetry.speed
         steering[carindex, packetnumber] = current_telemetry.steer
         gear[carindex, packetnumber] = current_telemetry.gear
         rev_lights_percent[carindex, packetnumber] = current_telemetry.rev_lights_percent
         surface_type[carindex, packetnumber] = current_telemetry.surface_type
         tire_inner_temp[carindex, packetnumber] = current_telemetry.tyres_inner_temperature
         tire_surface_temp[carindex, packetnumber] = current_telemetry.tyres_surface_temperature
   return { 
      "session_times": session_times, 
      "frame_identifiers": frame_identifiers, 
      "overall_frame_identifiers": overall_frame_identifiers, 
      "brake" : brake, 
      "throttle" : throttle, 
      "steering" : steering, 
      "speed" : speed, 
      "gear": gear,
      "rev_lights_percent": rev_lights_percent, 
      "tire_inner_temp": tire_inner_temp, 
      "tire_surface_temp": tire_surface_temp,
      "surface_type": surface_type
   }   
def numpifyMotionData(motion_data_msgs : list[drmsgs.TimestampedPacketMotionData], float_type=np.float32, int_type=np.int32) -> \
   dict[str, np.ndarray]:
   numpackets = len(motion_data_msgs)
   if numpackets<=0:
      raise ValueError("input list cannot be empty")
   numdrivers = len(motion_data_msgs[0].udp_packet.car_motion_data)
   session_times = np.empty(numpackets, dtype=float_type)
   frame_identifiers = np.empty(numpackets, dtype=int_type)
   overall_frame_identifiers = np.empty(numpackets, dtype=int_type)
   positions = np.empty([numdrivers, numpackets, 3], dtype=float_type)
   quaternions = np.empty([numdrivers, numpackets, 4], dtype=float_type)
   velocities = np.empty([numdrivers, numpackets, 3], dtype=float_type)
   accelerations = np.empty([numdrivers, numpackets, 3], dtype=float_type)

   for packetnumber in range(numpackets):
      session_times[packetnumber] = motion_data_msgs[packetnumber].udp_packet.header.session_time
      motion_packet = motion_data_msgs[packetnumber].udp_packet
      frame_identifiers[packetnumber] = motion_packet.header.frame_identifier
      try:
         overall_frame_identifiers[packetnumber] = motion_packet.header.overall_frame_identifier
      except AttributeError as e:
         overall_frame_identifiers[packetnumber] = motion_packet.header.frame_identifier
      for carindex in range(numdrivers):
         md : drmsgs.CarMotionData = motion_packet.car_motion_data[carindex]
         position, rotation = extractPose(motion_packet, car_index=carindex)
         positions[carindex, packetnumber] = position.astype(float_type)
         quaternions[carindex, packetnumber] = rotation.as_quat().astype(float_type)
         velocities[carindex, packetnumber] = extractVelocity(motion_packet, car_index=carindex)
         accelerations[carindex, packetnumber] = extractAcceleration(motion_packet, car_index=carindex)
         
   return { 
      "session_times": session_times, 
      "frame_identifiers": frame_identifiers, 
      "overall_frame_identifiers": overall_frame_identifiers, 
      "positions" : positions, 
      "quaternions" : quaternions, 
      "velocities": velocities, 
      "accelerations": accelerations
   }

def numpifyLapData(lap_data_msgs : list[drmsgs.TimestampedPacketLapData], float_type=np.float32, int_type=np.int32) -> \
   dict[str, np.ndarray]:
   numpackets = len(lap_data_msgs)
   if numpackets<=0:
      raise ValueError("input list cannot be empty")
   numdrivers = len(lap_data_msgs[0].udp_packet.lap_data)
   session_times = np.empty(numpackets, dtype=float_type)
   frame_identifiers = np.empty(numpackets, dtype=int_type)
   overall_frame_identifiers = np.empty(numpackets, dtype=int_type)
   lap_distances = np.empty([numdrivers, numpackets], dtype=float_type)
   total_distances = np.empty([numdrivers, numpackets], dtype=float_type)
   last_lap_times = np.empty([numdrivers, numpackets], dtype=float_type)
   current_lap_times = np.empty([numdrivers, numpackets], dtype=float_type)
   lap_numbers = np.empty([numdrivers, numpackets], dtype=int_type)
   result_status = np.empty([numdrivers, numpackets], dtype=int_type)
   driver_status = np.empty([numdrivers, numpackets], dtype=int_type)
   pit_status = np.empty([numdrivers, numpackets], dtype=int_type)
   pit_lane_timer_active = np.empty([numdrivers, numpackets], dtype=bool)
   
   for packetnumber in range(numpackets):
      session_times[packetnumber] = lap_data_msgs[packetnumber].udp_packet.header.session_time
      lap_packet : drmsgs.PacketLapData = lap_data_msgs[packetnumber].udp_packet
      frame_identifiers[packetnumber] = lap_packet.header.frame_identifier
      try:
         overall_frame_identifiers[packetnumber] = lap_packet.header.overall_frame_identifier
      except AttributeError as e:
         overall_frame_identifiers[packetnumber] = lap_packet.header.frame_identifier
      lap_data_array : List[drmsgs.LapData] = lap_packet.lap_data
      for carindex in range(numdrivers):
         lap_data : drmsgs.LapData = lap_data_array[carindex]
         lap_distances[carindex, packetnumber] = lap_data.lap_distance
         total_distances[carindex, packetnumber] = lap_data.total_distance
         if lap_data.last_lap_time_in_ms<=0:
            last_lap_times[carindex, packetnumber] = np.inf
         else:
            last_lap_times[carindex, packetnumber] = float(lap_data.last_lap_time_in_ms)/1000.0
         if lap_data.current_lap_time_in_ms<0:
            current_lap_times[carindex, packetnumber] = np.inf
         else:
            current_lap_times[carindex, packetnumber] = float(lap_data.current_lap_time_in_ms)/1000.0
         lap_numbers[carindex, packetnumber] = lap_data.current_lap_num
         result_status[carindex, packetnumber] = lap_data.result_status
         driver_status[carindex, packetnumber] = lap_data.driver_status
         pit_status[carindex, packetnumber] = lap_data.pit_status
         pit_lane_timer_active[carindex, packetnumber] = lap_data.pit_lane_timer_active>0
         
   return {
      "session_times" : session_times,
      "frame_identifiers": frame_identifiers, 
      "overall_frame_identifiers": overall_frame_identifiers, 
      "lap_distances" : lap_distances, 
      "total_distances" : total_distances, 
      "last_lap_times" : last_lap_times, 
      "current_lap_times" : current_lap_times, 
      "lap_numbers" : lap_numbers, 
      "result_status" : result_status, 
      "driver_status" : driver_status, 
      "pit_status" : pit_status, 
      "pit_lane_timer_active" : pit_lane_timer_active
   }