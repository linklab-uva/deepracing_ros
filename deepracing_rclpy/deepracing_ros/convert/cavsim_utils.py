import uva_iac_msgs.msg as uva_iac_msgs
import deepracing_models.math_utils as mu
import deepracing_models.math_utils.bezier as bezier
import torch
import sensor_msgs.msg as sensor_msgs
import nav_msgs.msg as navs_msgs
import ros2_numpy
import numpy as np
import scipy.interpolate

def cbc_to_track(control_points : torch.Tensor, delta_t : torch.Tensor, tsamp : torch.Tensor, 
                 lat_stdev_range : tuple[float, float], long_stdev_range : tuple[float, float], 
                 matrix_factory : mu.BezierMatrixFactory, matrix_factory_deriv : mu.BezierMatrixFactory, odom : navs_msgs.Odometry,  
                 matrix_factory_2ndderiv : mu.BezierMatrixFactory | None = None, track_id : int = 3, reputation : float = 1.0) \
    -> tuple[uva_iac_msgs.BatchTrack, uva_iac_msgs.BatchTrackPrediction]:
    batchtrack = uva_iac_msgs.BatchTrack()
    batchtrack.header = odom.header
    batchtrack.tracks = [uva_iac_msgs.Track(),]
    batchtrack.tracks[0].track_id = track_id
    batchtrack.tracks[0].reputation = reputation
    batchtrack.tracks[0].pose = odom.pose
    batchtrack.tracks[0].twist = odom.twist

    batchtrack_prediction = uva_iac_msgs.BatchTrackPrediction()
    batchtrack_prediction.header = odom.header
    batchtrack_prediction.track_predictions = [uva_iac_msgs.TrackPrediction(),]
    batchtrack_prediction.track_predictions[0].track_id = track_id
    batchtrack_prediction.track_predictions[0].reputation = reputation
    
    kbezier = control_points.shape[0] - 1
    control_points_deriv = kbezier * torch.diff(control_points, dim=-2)/delta_t[...,None,None]
    tstart = torch.cumsum(delta_t, 0) - delta_t
    Psamp, _ = mu.compositeBezierEval(tstart, delta_t, control_points, tsamp, matrix_factory)
    Vsamp, _ = mu.compositeBezierEval(tstart, delta_t, control_points_deriv, tsamp, matrix_factory_deriv)
    speedsamp : torch.Tensor = torch.linalg.vector_norm(Vsamp, dim=-1)
    tausamp : torch.Tensor = Vsamp/speedsamp[..., None]

    # if matrix_factory_2ndderiv is not None:
    #     control_points_2ndderiv = (kbezier-1) * torch.diff(control_points_deriv, dim=-2)/delta_t[...,None,None]
    #     Asamp, _ = mu.compositeBezierEval(tstart, delta_t, control_points_2ndderiv, tsamp, matrix_factory_2ndderiv)
    #     alongitudinal_samp : torch.Tensor = torch.linalg.vecdot(Asamp, tausamp)
    #     speedsquare = torch.square(speedsamp)
    #     delta_ssquare = (speedsquare[1:] - speedsquare[:-1])
    #     deltar = 0.5*(delta_ssquare/alongitudinal_samp[:-1])
    # else:
    #     deltar = speedsamp[:-1]*(tsamp[1:] - tsamp[:-1])
    tsamp_cpu = tsamp.cpu()
    speed_spline : scipy.interpolate.BSpline = scipy.interpolate.make_interp_spline(tsamp_cpu, speedsamp.cpu(), k=2)
    r_spline = speed_spline.antiderivative()
    # distances = torch.zeros_like(speedsamp)
    # distances[1:]= torch.cumsum(deltar, 0)
    distances = torch.as_tensor(r_spline(tsamp_cpu)).type_as(Psamp)

    lat_stdevs = torch.linspace(lat_stdev_range[0], lat_stdev_range[1], steps=tsamp.shape[0])
    long_stdevs = torch.linspace(long_stdev_range[0], long_stdev_range[1], steps=tsamp.shape[0])


    keys=["x", "y", "z", "xt", "yt", "zt", "speed", "distance", "time", "lat_uncertainty", "lon_uncertainty"]
    cloud_structured = np.zeros(Psamp.shape[0], dtype=[(k, np.float32) for k in keys])
    cloud_structured["x"] = Psamp[:, 0].cpu().numpy()
    cloud_structured["y"] = Psamp[:, 1].cpu().numpy()
    cloud_structured["xt"] = tausamp[:, 0].cpu().numpy()
    cloud_structured["yt"] = tausamp[:, 1].cpu().numpy()
    cloud_structured["speed"] = speedsamp.cpu().numpy()
    cloud_structured["distance"] = distances.cpu().numpy()
    cloud_structured["time"] = tsamp.cpu().numpy()
    cloud_structured["lat_uncertainty"] = lat_stdevs.cpu().numpy()
    cloud_structured["lon_uncertainty"] = long_stdevs.cpu().numpy()
    
    batchtrack_prediction.track_predictions[0].position_history = \
        ros2_numpy.msgify(sensor_msgs.PointCloud2, cloud_structured)
    batchtrack_prediction.track_predictions[0].position_history.header = odom.header

    return batchtrack, batchtrack_prediction