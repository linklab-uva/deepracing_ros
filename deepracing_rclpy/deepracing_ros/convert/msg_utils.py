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

def extractAcceleration(packet : drmsgs.PacketMotionData , car_index = -1) -> np.ndarray:
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
   acceleration = 9.8*np.array( (motion_data.g_force_longitudinal, motion_data.g_force_lateral, motion_data.g_force_vertical), dtype=np.float64)
   return acceleration 

def extractAngularVelocity(packet : drmsgs.PacketMotionData) -> np.ndarray:
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

   leftdir : geo_msgs.Vector3 = motion_data.world_left_dir.vector
   forwarddir : geo_msgs.Vector3 = motion_data.world_forward_dir.vector

   leftvector = np.array((leftdir.x, leftdir.y, leftdir.z), dtype=np.float64)
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
