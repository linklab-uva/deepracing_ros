import time
from copy import deepcopy
import numpy as np
import torch
from scipy.spatial.transform import Rotation
from deepracing_models.math_utils import bezier
from deepracing_msgs.msg import CompositeBezierCurve
import rclpy
import rclpy.time
import rclpy.client
import rclpy.qos
import rclpy.publisher
import rclpy.subscription
import rclpy.client
import rclpy.parameter
import rosbag2_interfaces.srv, rosbag2_interfaces.msg

import deepracing_models.math_utils.convert as math_C
import deepracing_models.math_utils as mu, deepracing_ros.convert as C
from deepracing_models.math_utils.bayesian_filtering import BayesianFilter, ParticleNoiser
from deepracing_models.math_utils.bounds_checking import BoundsChecker
from deepracing_models.math_utils.statistics import CollisionProbabilityEstimator
from deepracing_models.math_utils.dynamics import ExceedLimitsProbabilityEstimator
from deepracing_ros.controls.path_server_ros import PathServerROS
from deepracing_rclpy.dbf_overtaking import dbf_overtaking
import scipy.interpolate
import threading
import std_msgs.msg
import sensor_msgs.msg
import builtin_interfaces.msg
import ros2_numpy

class DBFOvertakingPathServer(PathServerROS):
    STATE_PARAMETER_NAME="state"
    def __init__(self):
        super(DBFOvertakingPathServer, self).__init__()
        self.get_logger().info("Hello Path Server! I live in namespace: %s" % (self.get_namespace()))

        self.declare_parameter(DBFOvertakingPathServer.STATE_PARAMETER_NAME, value="CREATED")

        self.param_listener = dbf_overtaking.ParamListener(self)
        # dbf_overtaking.ParamListener.update()
        self.params = self.param_listener.get_params()

        brake_factor = long_accel_factor = lat_accel_factor = 1.0 #self.params.timescale


        brake_speeds = (1.0 + 0.000)*torch.as_tensor([-1.0,    0.00,    25.190,  40.192,  64.544,  75.197,  89.330,  1000.0])
        max_brakes = brake_factor*torch.as_tensor( [-14.574,  -14.574, -14.574, -17.701, -21.424, -23.359, -25.593, -25.593])
        self.declare_parameter("brake_speeds", value=brake_speeds.cpu().numpy().tolist())
        self.declare_parameter("max_brakes", value=max_brakes.cpu().numpy().tolist())

        long_accel_speeds = (1.0 + 0.000)*torch.as_tensor(    [-1.0,    0.0,    24.102,  40.192,  48.237,  59.325, 75.850, 91.069, 92.5,  1000.0]) 
        max_long_accels = long_accel_factor*torch.as_tensor([ 14.162,   14.162, 14.162,  12.971,  12.375,  9.546,  4.484,  0.464,  0.0,   0.0])
        self.declare_parameter("long_accel_speeds", value=long_accel_speeds.cpu().numpy().tolist())
        self.declare_parameter("max_long_accels", value=max_long_accels.cpu().numpy().tolist())

        lat_accel_speeds = (1.0 + 0.000)*torch.as_tensor([-1.00,   0.0,    19.0,   75.850,  91.069,  1000.0]) 
        max_lat_accels = lat_accel_factor*torch.as_tensor([ 12.224,   12.224, 16.224, 35.156,  40.218,  40.218])
        self.declare_parameter("lat_accel_speeds", value=lat_accel_speeds.cpu().numpy().tolist())
        self.declare_parameter("max_lat_accels", value=max_lat_accels.cpu().numpy().tolist())
        self.overtaking_curve = None
        self.overtaking_dT = None

        
        self.opponent_curve_mutex = threading.Semaphore()
        self.opponent_curve_msg : CompositeBezierCurve | None = None
        self.opponent_bcurve_sub : rclpy.subscription.Subscription = self.create_subscription(CompositeBezierCurve, "opponent_curve", self.opponent_curve_CB, rclpy.qos.qos_profile_sensor_data)
        self.cloud_pub : rclpy.publisher.Publisher =  self.create_publisher(sensor_msgs.msg.PointCloud2, "overtaking_curves", 1)
        self.pathswitch_pub : rclpy.publisher.Publisher =  self.create_publisher(std_msgs.msg.String, "switch_path", 1)
        self.overtake_begin_pub : rclpy.publisher.Publisher =  self.create_publisher(builtin_interfaces.msg.Time, "overtake_begin", 1)
        self.overtake_end_pub : rclpy.publisher.Publisher =  self.create_publisher(builtin_interfaces.msg.Time, "overtake_end", 1)
        self.composite_bcurve_pub : rclpy.publisher.Publisher = self.create_publisher(CompositeBezierCurve, "bcurvesout", 1)
        self.opponent_composite_bcurve_pub : rclpy.publisher.Publisher = self.create_publisher(CompositeBezierCurve, "paired_opponent_curve", 1)
        # self.composite_bcurve_pub : rclpy.publisher.Publisher = None #self.create_publisher(CompositeBezierCurve, "oraclecompositebeziercurves", 1)
        # bagrecordername_param = self.declare_parameter("bag_recorder_name", value="/rosbag2_recorder")
        # bagrecordername = bagrecordername_param.get_parameter_value().string_value
        # self.pause_service : rclpy.client.Client = self.create_client(rosbag2_interfaces.srv.Pause, "%s/pause" % (bagrecordername,))
        # self.unpause_service : rclpy.client.Client = self.create_client(rosbag2_interfaces.srv.Resume, "%s/resume" % (bagrecordername,))
    def opponent_curve_CB(self, msg : CompositeBezierCurve):
        if not self.opponent_curve_mutex.acquire(timeout=0.1):
            raise ValueError("Unable to acquire opponent_curve_mutex")
        self.opponent_curve_msg = msg
        self.opponent_curve_mutex.release()
    def initialize(self, raceline_structured : np.ndarray, widthmap_structured : np.ndarray, innerbound_structured : np.ndarray, outerbound_structured : np.ndarray):
        stateparam = rclpy.Parameter(DBFOvertakingPathServer.STATE_PARAMETER_NAME, rclpy.Parameter.Type.STRING, "INITIALIZING")
        self.set_parameters([stateparam,])
        torch.set_float32_matmul_precision("high")
        device = torch.device("cuda:%d" % self.params.gpu if self.params.gpu>=0 else "cpu")
        self.get_logger().info("Building Raceline Helpers")
        line_all_points = torch.as_tensor(np.stack([raceline_structured["x"], raceline_structured["y"]], axis=1), dtype=torch.float32, device=device)
        times_in = raceline_structured["time"].astype(np.float64)
        interp_spline : scipy.interpolate.BSpline = \
            scipy.interpolate.make_interp_spline(times_in, line_all_points.cpu().numpy(), k=2, bc_type="periodic")
        interp_times = np.linspace(times_in[0], times_in[-1], num=int(round(times_in[-1].item()/0.075)))
        interp_spline_points = interp_spline(interp_times)
        interp_spline_speeds = np.linalg.norm(interp_spline(interp_times, nu=1), ord=2.0, axis=1)
        line_all_speeds = torch.as_tensor(interp_spline_speeds).double()
        # line_all_speeds = torch.as_tensor(raceline_structured["speed"]).double()
        # line_all_points_cast = torch.zeros_like(line_all_points).type_as(line_all_speeds)
        _raceline_helper_ : mu.RacelineHelper = mu.RacelineHelper.from_closed_path(
            torch.as_tensor(interp_spline_points).type_as(line_all_speeds), self.params.timescale*line_all_speeds,
            0.5
        ).to(tensor=line_all_points)
        # self.fullspeed_raceline_helper : mu.RacelineHelper = mu.RacelineHelper.from_closed_path(
        #     line_all_points.cpu().double(), line_all_speeds,
        #     0.5
        # ).to(tensor=line_all_points)
        self.get_logger().info("Built Raceline Helpers")
        self.get_logger().info("Building Bounds Checker")
        shrink_factor = 1.0
        centerline_dense = torch.as_tensor(np.stack([widthmap_structured[k] for k in ["x", "y"]], axis=1)).type_as(line_all_speeds)
        _centerline_helper_ = mu.SimplePathHelper.from_closed_path(centerline_dense, 0.5).to(tensor=line_all_points)
        left_widths = shrink_factor*torch.as_tensor(widthmap_structured["ob_distance"]).type_as(line_all_speeds) - 0.2*self.params.car_dims.width
        right_widths = shrink_factor*torch.as_tensor(widthmap_structured["ib_distance"]).type_as(line_all_speeds) + 0.2*self.params.car_dims.width
        _bounds_checker_ = BoundsChecker(gauss_order=self.params.bounds_gauss.order, dT=self.params.time_horizon, stdev=self.params.bounds_gauss.stdev,
            dr_samp=self.params.bounds_gauss.dr_samp, alpha=self.params.bounds_gauss.alpha,
            left_widths=left_widths, right_widths=right_widths, refline_points=centerline_dense,
            newton_iterations=self.params.bounds_newton.iterations, newton_stepsize=self.params.bounds_newton.stepsize, max_step=self.params.bounds_newton.max_step
        ).eval().to(tensor=line_all_points)
        self.get_logger().info("Built Bounds Checker")
        nd = torch.distributions.Normal(0.0, 1.0)
        scaledown = nd.cdf(torch.as_tensor(2.0).sqrt()).item()
        self.get_logger().info("Building Dynamics Checker")
        # brake_factor = long_accel_factor = lat_accel_factor = self.params.timescale
        _dynamic_violation_estimator_ = ExceedLimitsProbabilityEstimator(
            torch.as_tensor(self.get_parameter("brake_speeds").value),
            scaledown*torch.as_tensor(self.get_parameter("max_brakes").value),

            torch.as_tensor(self.get_parameter("long_accel_speeds").value),
            scaledown*torch.as_tensor(self.get_parameter("max_long_accels").value),

            torch.as_tensor(self.get_parameter("lat_accel_speeds").value),
            scaledown*torch.as_tensor(self.get_parameter("max_lat_accels").value),

            gauss_order=self.params.dynamics_gauss.order,
            stdev=self.params.dynamics_gauss.stdev,
            alpha=self.params.dynamics_gauss.alpha,
            newton_iterations=self.params.dynamics_newton.iterations,
            newton_stepsize=self.params.dynamics_newton.stepsize,
            max_step=self.params.dynamics_newton.max_step,
            dT=self.params.time_horizon
        ).to(tensor=line_all_points)
        self.get_logger().info("Built Dynamics Checker")

        self.get_logger().info("Building Collision Checker")
        car_length, car_width = self.params.car_dims.length, self.params.car_dims.width
        long_buffer = car_length*self.params.buffer_factor.longitudinal
        lat_buffer = car_width*self.params.buffer_factor.lateral
        _collision_probability_estimator_ : CollisionProbabilityEstimator = CollisionProbabilityEstimator(
            self.params.collision_gauss.order_time, self.params.time_horizon, self.params.collision_gauss.order_space, lat_buffer, long_buffer, 
            gamma=self.params.collision_gauss.gamma, alpha=self.params.collision_gauss.alpha
        ).to(tensor=line_all_points)
        eta_scaled = (_collision_probability_estimator_.gl1d.eta.detach())/self.params.time_horizon
        boxpoints_target_01 = 0.5*torch.stack([
            torch.as_tensor([0.0, 0.0]),
            torch.as_tensor([car_length, 0.0]),
            torch.as_tensor([-car_length, 0.0]),
            torch.as_tensor([0.0, car_width]),
            torch.as_tensor([0.0, -car_width]),
            torch.as_tensor([car_length, car_width]),
            torch.as_tensor([-car_length, car_width]),
            torch.as_tensor([car_length, -car_width]),
            torch.as_tensor([-car_length, -car_width]),
            # torch.as_tensor([0.5*car_length, car_width]),
            # torch.as_tensor([0.5*car_length, -car_width]),
            # torch.as_tensor([-0.5*car_length, car_width]),
            # torch.as_tensor([-0.5*car_length, -car_width]),
        ], dim=0).type_as(line_all_points)
        long_stdev_range, lat_stdev_range = self.params.stdev_range.longitudinal, self.params.stdev_range.lateral
        logtwopi = float(np.log(2.0*np.pi))
        TV_stdevs = torch.empty(self.params.collision_gauss.order_time, boxpoints_target_01.shape[-1]).type_as(boxpoints_target_01)
        TV_stdevs[:,0] = ((long_stdev_range[1]-long_stdev_range[0])*eta_scaled + long_stdev_range[0])#[:,None].expand_as(TV_stdevs[...,0])
        TV_stdevs[:,1] = ((lat_stdev_range[1]-lat_stdev_range[0])*eta_scaled + lat_stdev_range[0])#[:,None].expand_as(TV_stdevs[...,1])
        TV_stdevs = TV_stdevs.unsqueeze(1).expand(TV_stdevs.shape[0], boxpoints_target_01.shape[0], TV_stdevs.shape[1]).clone()
        TV_stdevs = TV_stdevs[None].expand(self.params.Nparticles, *TV_stdevs.shape).clone()
        TV_logstdevs = (logtwopi + torch.log(TV_stdevs.sum(dim=-1)))
        TV_invdiag = torch.diag_embed(1.0/TV_stdevs)
        self.get_logger().info("TV_stdevs.shape: %s" % (str(TV_stdevs.shape),))
        self.get_logger().info("TV_logstdevs.shape: %s" % (str(TV_logstdevs.shape),))
        self.get_logger().info("TV_invdiag.shape: %s" % (str(TV_invdiag.shape),))
        self.get_logger().info("Long stdevs: %s" % (str(TV_stdevs[0,:,0,0]),))
        self.get_logger().info("Lat stdevs: %s" % (str(TV_stdevs[0,:,0,1]),))
        self.get_logger().info("Built Collision Checker")

        self.get_logger().info("Building Overall Filter")
        self.boxpoints_target_01 = boxpoints_target_01
        self.TV_stdevs = TV_stdevs
        self.TV_invdiag = TV_invdiag
        self.TV_logstdevs = TV_logstdevs
        self.boxpoints_target_01 = boxpoints_target_01
        _overall_filter_ = BayesianFilter(
            collision_probability_estimator=_collision_probability_estimator_,
            dynamic_violation_estimator=_dynamic_violation_estimator_,
            bounds_checker=_bounds_checker_
        ).to(tensor=line_all_points)
        
        if self.params.controlpoint_noise.longitudinal < 0.0:
            longitudinal_controlpoint_noise = 0.5*self.params.controlpoint_noise.lateral
        else:
            longitudinal_controlpoint_noise = self.params.controlpoint_noise.longitudinal
        if self.params.controlpoint_noise.final < 0.0:
            final_controlpoint_noise=1.5*longitudinal_controlpoint_noise
        else:
            final_controlpoint_noise=self.params.controlpoint_noise.final
        _particle_noiser_  = ParticleNoiser(self.params.controlpoint_noise.lateral, longitudinal_controlpoint_noise, final_controlpoint_noise, _raceline_helper_).to(tensor=line_all_points)
        
        tdelta = torch.linspace(0.0, self.params.time_horizon, steps=60).type_as(line_all_points)[None]
        rtarget, Ptarget, Vtarget, _ = _raceline_helper_(t=(0.875*tdelta) + 2.0)
        Targetvehicle_curve, Curvefit_tswitch = mu.compositeBezierFit(tdelta, Ptarget, self.params.Nsegments, 
                                                                        dYdT_0=Vtarget[:,0],
                                                                        dYdT_f=Vtarget[:,-1],
                                                                        Y_0=Ptarget[:,0],
                                                                        Y_f=Ptarget[:,-1],
                                                                        kbezier=self.params.kbezier, 
                                                                        constraint_level=2)
        Curveparticle_tstart = Curvefit_tswitch[:,:-1].expand(self.params.Nparticles, Curvefit_tswitch.shape[-1]-1).clone()
        Curveparticle_dT = torch.diff(Curvefit_tswitch, dim=-1).expand(self.params.Nparticles, Curvefit_tswitch.shape[-1]-1).clone()
        Targetvehicle_curve_deriv = self.params.kbezier*torch.diff(Targetvehicle_curve, dim=-2)/Curveparticle_dT[[0,], :, None, None]
        tnodes = _collision_probability_estimator_.gl1d.eta.detach().unsqueeze(0)
        (TV_positions,), _ = mu.compositeBezierEval(Curveparticle_tstart[[0,]], Curveparticle_dT[[0,]], Targetvehicle_curve, tnodes, _overall_filter_.matrix_factory) 
        (TV_velocities,), _ = mu.compositeBezierEval(Curveparticle_tstart[[0,]], Curveparticle_dT[[0,]], Targetvehicle_curve_deriv, tnodes, _overall_filter_.derivative_matrix_factory) 
        TV_tangents = TV_velocities/torch.linalg.vector_norm(TV_velocities, dim=-1, keepdim=True)
        TV_normals = TV_tangents[...,[1,0]]
        TV_normals[...,0]*=-1.0
        TV_rotmats = torch.stack([TV_tangents, TV_normals], dim=-1)
        self.get_logger().info("TV_positions.shape: "  +  str(TV_positions.shape))
        self.get_logger().info("TV_rotmats.shape: "  +  str(TV_rotmats.shape))
        TV_box_positions = (TV_rotmats@boxpoints_target_01[None].transpose(-2,-1)).transpose(-2,-1) + TV_positions[:,None]
        TV_box_positions = TV_box_positions[None].expand(self.params.Nparticles, *TV_box_positions.shape)
        TV_eigvecs = TV_rotmats[None,:,None].expand(self.params.Nparticles, TV_positions.shape[0], boxpoints_target_01.shape[0], 2, 2)#.clone()
        TV_stdev_inv_matrix : torch.Tensor = TV_invdiag@TV_eigvecs.transpose(-2,-1)#.type_as(rotmats)
        self.get_logger().info("TV_box_positions.shape: "  +  str(TV_box_positions.shape))
        self.get_logger().info("TV_eigvecs.shape: "  +  str(TV_eigvecs.shape))
        self.get_logger().info("TV_stdev_inv_matrix.shape: "  +  str(TV_stdev_inv_matrix.shape))


        rego, Pego, Vego, _ = _raceline_helper_(t=tdelta + 1.5)
        Curveparticles, _ = mu.compositeBezierFit(tdelta, Pego, self.params.Nsegments, 
                                                                        dYdT_0=Vego[:,0],
                                                                        dYdT_f=Vego[:,-1],
                                                                        Y_0=Pego[:,0],
                                                                        Y_f=Pego[:,-1],
                                                                        kbezier=self.params.kbezier, 
                                                                        constraint_level=2)
        Curveparticles = Curveparticles.expand(self.params.Nparticles, *Curveparticles.shape[1:]).clone()
        rfinal = rego[:,-1].tile(self.params.Nparticles)
        rfinal_min = (rtarget[0,-1] + 2.5*car_length)
        self.get_logger().info("Built Overall Filter")
        compile_backend = self.params.compile_backend
        compile_modules = (len(compile_backend)>0) and (not compile_backend=="none")
        backend = compile_backend if compile_modules else "inductor"
        mode="max-autotune-no-cudagraphs" if backend=="inductor" else None
        self.get_logger().info("Compiling Overall Filter")
        # torch.set_float32_matmul_precision('high')
        _closest_point_finder_ = bezier.ClosestPointFinder(order = self.params.kbezier).to(tensor=line_all_points)
        self.raceline_helper : mu.RacelineHelper = torch.compile(_raceline_helper_, fullgraph=True, dynamic=True, mode=mode, backend=backend, disable=(not compile_modules))
        # self.t_of_r : mu.TofRHelper = torch.compile(mu.TofRHelper(_raceline_helper_.__r_of_t__.control_points.detach().clone(), _raceline_helper_.__times_in__.detach().clone()), fullgraph=True, dynamic=True, mode=mode, backend=backend, disable=(not compile_modules))
        self.particle_noiser : ParticleNoiser = torch.compile(_particle_noiser_, fullgraph=True, dynamic=False, mode=mode, backend=backend, disable=(not compile_modules))
        self.overall_filter : BayesianFilter = torch.compile(_overall_filter_, fullgraph=True, dynamic=False, mode=mode, backend=backend, disable=(not compile_modules))
        self.centerline_helper : mu.SimplePathHelper = torch.compile(_centerline_helper_, fullgraph=True, dynamic=True, mode=mode, backend=backend, disable=(not compile_modules))
        self.closest_point_finder : mu.SimplePathHelper = torch.compile(_closest_point_finder_, fullgraph=True, dynamic=False, mode=mode, backend=backend, disable=(not compile_modules))
        warmup_runs = 20
        warmup_times = []
        tstart = torch.cumsum(Curveparticle_dT[0], 0) - Curveparticle_dT[0]
        tfit = torch.linspace(0.0, self.params.time_horizon, steps=60).type_as(Curveparticles)
        for i in range(warmup_runs):
            rfinal_ = rfinal + 50.0*torch.randn_like(rfinal)
            TV_box_positions_ = TV_box_positions + 5.0*torch.randn_like(TV_box_positions)
            tick = time.time()
            Curveparticles_ = Curveparticles + 5.0*torch.randn_like(Curveparticles)#Curveparticle_tstart + 0.025*torch.randn_like(Curveparticle_tstart)
            Curveparticles_, rfinal_ = self.particle_noiser(Curveparticles_, Curveparticle_dT, rfinal_, rfinal_min)
            # faketstart -= faketstart[:,:,[0,]]
            # faketdt = Curveparticle_dT + 0.025*torch.randn_like(Curveparticle_dT)
            # faketstart = torch.zeros_like(faketdt)
            # faketstart[:,1:]=torch.cumsum(faketdt,1)[:,1:]
            self.overall_filter(Curveparticles_, Curveparticle_tstart, Curveparticle_dT, 
                        TV_box_positions_ + torch.zeros_like(TV_box_positions_), TV_stdev_inv_matrix + torch.zeros_like(TV_stdev_inv_matrix), TV_logstdevs + torch.zeros_like(TV_logstdevs))
            # _, Pfinal, Vfinal, _ = self.raceline_helper(r=rfinal[[0,]], deriv=True)
            Pquery = TV_positions[[0,]]
            r, _, _, _ = self.raceline_helper(t=tfit + 10.0 + torch.randn(1).item())
            self.raceline_helper(r=r + 10.0 + torch.randn(1).item())
            self.raceline_helper.closest_point_approximate(Pquery + torch.randn_like(Pquery), newton_iterations=3)
            self.raceline_helper.t_of_r(rfinal_[[0,]])

            istart = torch.randint(0, centerline_dense.shape[0], (1,)).item()
            idx_grab = torch.arange(istart, istart+400, step=1, dtype=torch.int64)%(centerline_dense.shape[0])
            pquery = (centerline_dense[idx_grab] + 4.0*torch.randn_like(centerline_dense[idx_grab])).type_as(line_all_points)

            r, _, _, _ = self.centerline_helper.closest_point_approximate(pquery, newton_iterations=3)
            self.centerline_helper(r)

            curve_particle_withz = torch.cat([Curveparticles[0], torch.zeros_like(Curveparticles[0,...,[0,]])], dim=-1)
            curve_particle_withz_flat = curve_particle_withz.reshape(-1, curve_particle_withz.shape[-1])[1:-1]
            iquery = torch.randint(0, curve_particle_withz_flat.shape[0], (1,)).item()
            pquery = curve_particle_withz_flat[iquery].clone()
            pquery += 4.0*torch.randn_like(pquery)
            # self.get_logger().info("pquery.shape: " + str(pquery.shape))
            # self.get_logger().info("curve_particle_withz.shape: " + str(curve_particle_withz.shape))
            # self.get_logger().info("tstart.shape: " + str(tstart.shape))
            # self.get_logger().info("Curveparticle_dT[0].shape: " + str(Curveparticle_dT[0].shape))
            # t, P, V, _ = self.closest_point_finder(tstart, Curveparticle_dT[0], curve_particle_withz, pquery)
            
            tock = time.time()
            warmup_times.append(tock-tick)
        self.tfit = tfit
        self.Curveparticles = None
        self.Curveparticle_tstart = Curveparticle_tstart
        self.Curveparticle_dT = Curveparticle_dT
        warmup_times = torch.as_tensor(warmup_times, dtype=torch.float64)
        self.get_logger().info("Compiled Overall Filter")
        self.get_logger().info("warmup_times: " + str(warmup_times))
        stateparam = rclpy.Parameter(DBFOvertakingPathServer.STATE_PARAMETER_NAME, rclpy.Parameter.Type.STRING, "PLANNING")
        self.set_parameters([stateparam,])

    
    def getTrajectory(self):
        now = self.get_clock().now()
        if self.current_odom is None:
            self.get_logger().error("No odom yet")
            return
        state : str = self.get_parameter(DBFOvertakingPathServer.STATE_PARAMETER_NAME).value
        if (state=="PLANNING"):
            self.handleStatePlanning(now)
        elif (state=="OVERTAKING"):
            self.handleStateOvertaking(now)
        elif (state=="IDLE"):
            self.handleStateIdle(now)

    def handleStatePlanning(self, now : rclpy.time.Time):

        current_pose_msg = deepcopy(self.current_odom.pose.pose)
        current_vel_msg = deepcopy(self.current_odom.twist.twist)
        current_rot = Rotation.from_quat([0.0, 0.0, current_pose_msg.orientation.z, current_pose_msg.orientation.w])
        current_rotmat = torch.as_tensor(current_rot.as_matrix()[0:2,0:2]).type_as(self.Curveparticle_tstart)
        current_position_msg = current_pose_msg.position
        current_position = torch.as_tensor([current_position_msg.x, current_position_msg.y]).type_as(self.Curveparticle_tstart)
        current_velocity = (current_rotmat@torch.as_tensor([[current_vel_msg.linear.x,], [current_vel_msg.linear.y,]]).type_as(self.Curveparticle_tstart))[:,0]
        
        rclosest, _, _, _ = self.raceline_helper.closest_point_approximate(current_position[None], newton_iterations=3)
        tclosest = self.raceline_helper.t_of_r(rclosest).item()
        if self.Curveparticles is None:
            tfit = torch.linspace(tclosest, tclosest + self.params.time_horizon, steps=40).type_as(self.Curveparticle_tstart)
        rfit, pfit, vfit, _ = self.raceline_helper(t=1.075*self.tfit + tclosest)
        Curveparticles, _ = mu.compositeBezierFit(self.tfit[None], pfit[None], numsegments=self.params.Nsegments, kbezier=self.params.kbezier,
                                                    Y_0=current_position[None], dYdT_0=current_velocity[None], 
                                                    Y_f=pfit[[-1,]], dYdT_f=vfit[[-1],],
                                                    constraint_level=2 )
        rfinal = rfit[[-1,]].expand(self.params.Nparticles).clone()
        self.Curveparticles = Curveparticles.expand(self.params.Nparticles, *Curveparticles.shape[1:]).clone()
        if self.opponent_curve_msg is None:
            self.get_logger().error("No opponent curve")
        else:
            if not self.opponent_curve_mutex.acquire(timeout=0.1):
                raise ValueError("Unable to acquire opponent_curve_mutex")
            opponent_curve_msg = deepcopy(self.opponent_curve_msg)
            self.opponent_curve_mutex.release()
            TV_curve_dT, Targetvehicle_curve  = C.fromCompositeBezierCurveMsg(opponent_curve_msg, dtype=self.tfit.dtype, device=self.tfit.device)
            Targetvehicle_curve = Targetvehicle_curve[...,[0,1]]
            # print(Targetvehicle_curve.shape)
            # print(TV_curve_dT.shape)
            TV_curve_tstart = torch.cumsum(TV_curve_dT, 0)-TV_curve_dT[0]
            # print(TV_curve_tstart.shape)
            Targetvehicle_curve_deriv = self.params.kbezier*torch.diff(Targetvehicle_curve, dim=-2)/TV_curve_dT[:, None, None]
            tnodes = self.overall_filter.collision_probability_estimator.gl1d.eta
            # print(tnodes.shape)
            TV_positions, _ = mu.compositeBezierEval(TV_curve_tstart, TV_curve_dT, Targetvehicle_curve, tnodes, self.overall_filter.matrix_factory) 
            TV_rinitial, _, _, _ = self.raceline_helper.closest_point_approximate(Targetvehicle_curve[0,[0,]], newton_iterations=3)
            TV_rfinal, _, _, _ = self.raceline_helper.closest_point_approximate(Targetvehicle_curve[-1,[-1,]], newton_iterations=3)
            if TV_rinitial[0]>TV_rfinal[0]:
                TV_rfinal+=self.raceline_helper.__arclengths_in__[-1]
                rfinal[rfinal<(0.25*self.raceline_helper.__arclengths_in__[-1])]+=self.raceline_helper.__arclengths_in__[-1]
            rfinal_min = TV_rfinal[0] + 3.0*self.params.car_dims.length
            TV_velocities, _ = mu.compositeBezierEval(TV_curve_tstart, TV_curve_dT, Targetvehicle_curve_deriv, tnodes, self.overall_filter.derivative_matrix_factory) 
            TV_tangents : torch.Tensor = TV_velocities/torch.linalg.vector_norm(TV_velocities, dim=-1, keepdim=True)
            TV_normals = TV_tangents[:,[1,0]].clone()
            TV_normals[:,0]*=-1.0
            TV_rotmats = torch.stack([TV_tangents, TV_normals], dim=-1)
            TV_box_positions = (TV_rotmats@self.boxpoints_target_01.transpose(-2,-1)).transpose(-2,-1) + TV_positions[:,None]
            
            TV_eigvecs = TV_rotmats[None,:,None].expand(self.params.Nparticles, TV_positions.shape[0], self.boxpoints_target_01.shape[0], 2, 2)#.clone()
            TV_stdev_inv_matrix : torch.Tensor = self.TV_invdiag@TV_eigvecs.transpose(-2,-1)
            TV_box_positions = TV_box_positions[None].expand(self.params.Nparticles, *TV_box_positions.shape)

            tick = time.time()
            dbf_curve, dbf_rfinal = self.attempt_dbf(
                self.Curveparticles, self.Curveparticle_tstart, self.Curveparticle_dT, rfinal, rfinal_min,
                    TV_box_positions, TV_stdev_inv_matrix
            )
            tock = time.time()
            if (dbf_curve is not None) and (dbf_rfinal is not None):
                Vfinal = self.params.kbezier*(dbf_curve[-1,-1] - dbf_curve[-1,-2])/self.Curveparticle_dT[0,-1]
                tsplice = self.raceline_helper.t_of_r(dbf_rfinal[None]).item()
                
                textra = torch.linspace(tsplice, tsplice+self.params.time_horizon, steps=10*self.params.Nsegments).type_as(dbf_rfinal)
                _, pextra, _, _ = self.raceline_helper(t=textra)
                curve_extra, curve_extra_tswitch = mu.compositeBezierFit(textra-textra[0], pextra, self.params.Nsegments, Y_0=dbf_curve[-1,-1], dYdT_0=Vfinal, kbezier=self.params.kbezier, constraint_level=2)
                curve_extra_dT = torch.diff(curve_extra_tswitch, dim=0)

                overtaking_curve = torch.cat([dbf_curve, curve_extra], dim=0)
                overtaking_curve_fakez = torch.zeros_like(overtaking_curve[...,[0,]])
                self.overtaking_curve = torch.cat([overtaking_curve, overtaking_curve_fakez], dim=-1)#.clone()
                self.overtaking_dT = torch.cat([self.Curveparticle_dT[0], curve_extra_dT], dim=0)#.clone()

                self.overtaking_curve_deriv = self.params.kbezier*torch.diff(self.overtaking_curve, dim=-2)/self.overtaking_dT[:, None, None]
                self.overtaking_curve_2ndderiv = (self.params.kbezier-1)*torch.diff(self.overtaking_curve_deriv, dim=-2)/self.overtaking_dT[:, None, None]

                self.overtaking_tstart = torch.cumsum(self.overtaking_dT, 0) - self.overtaking_dT
                self.overtaking_tsamp = torch.linspace(0.0, (self.overtaking_tstart[-1] + self.overtaking_dT[-1]).item(), steps=2000).type_as(self.overtaking_tstart)
                self.overtake_start_time = now
                # self.get_logger().info("self.overtaking_tsamp.shape: " + str(self.overtaking_tsamp.shape))
                (self.overtaking_psamp,), idxbuckets = mu.compositeBezierEval(self.overtaking_tstart[None], self.overtaking_dT[None], self.overtaking_curve[None], self.overtaking_tsamp[None], self.overall_filter.matrix_factory)
                (self.overtaking_vsamp,), _ = mu.compositeBezierEval(self.overtaking_tstart[None], self.overtaking_dT[None], self.overtaking_curve_deriv[None], self.overtaking_tsamp[None], self.overall_filter.derivative_matrix_factory, idxbuckets=idxbuckets)
                (self.overtaking_asamp,), _ = mu.compositeBezierEval(self.overtaking_tstart[None], self.overtaking_dT[None], self.overtaking_curve_2ndderiv[None], self.overtaking_tsamp[None], self.overall_filter.second_derivative_matrix_factory, idxbuckets=idxbuckets)

            

                msgout = C.toCompositeBezierCurveMsg(self.overtaking_dT, self.overtaking_curve)
                msgout.header.frame_id="map"
                msgout.header.stamp = self.get_clock().now().to_msg()
                
                
                opponent_curve_msg.header=msgout.header
                self.composite_bcurve_pub.publish(msgout)
                self.opponent_composite_bcurve_pub.publish(opponent_curve_msg)
                matrix_factories = {
                    self.params.kbezier : self.overall_filter.matrix_factory,
                    self.params.kbezier-1 : self.overall_filter.derivative_matrix_factory,
                    self.params.kbezier-2 : self.overall_filter.second_derivative_matrix_factory
                }
                deltat_ros = self.get_clock().now() - now
                tstart : float = deltat_ros.nanoseconds*1e-9
                if tstart>0.15:
                    self.get_logger().warn("DBF took too long to compute: %f seconds. Not context switching" % (tstart,))
                    return
                tsamp = torch.linspace(tstart, tstart+1.6, steps=41).type_as(self.overtaking_curve)
                numpy_cloud = math_C.to_cavsim_cloud(self.overtaking_curve, self.overtaking_dT, tsamp, matrix_factories,) 
                cloud_msg = ros2_numpy.msgify(sensor_msgs.msg.PointCloud2, numpy_cloud)
                cloud_msg.header.frame_id="map"
                cloud_msg.header.stamp = now.to_msg()
                self.cloud_pub.publish(cloud_msg)
                self.pathswitch_pub.publish(std_msgs.msg.String(data="graph"))
                
                stateparam = rclpy.Parameter(DBFOvertakingPathServer.STATE_PARAMETER_NAME, rclpy.Parameter.Type.STRING, "OVERTAKING")
                self.set_parameters([stateparam,])
                self.overtake_begin_pub.publish(self.get_clock().now().to_msg())
            else:
                self.get_logger().debug("DBF algorithm did not converge in %f seconds" % (tock-tick,))
    def handleStateOvertaking(self, now : rclpy.time.Time):
        self.get_logger().debug("Handling Overtaking State")
        # current_odom = deepcopy(self.current_odom)
        # current_pose_msg = current_odom.pose.pose
        # current_vel_msg = current_odom.twist.twist
        # current_rot = Rotation.from_quat([current_pose_msg.orientation.x, current_pose_msg.orientation.y, current_pose_msg.orientation.z, current_pose_msg.orientation.w])
        # current_rotmat = torch.as_tensor(current_rot.as_matrix()).type_as(self.overtaking_curve)
        # current_vel_local = torch.as_tensor([current_vel_msg.linear.x, current_vel_msg.linear.y, current_vel_msg.linear.z]).type_as(self.overtaking_curve)
        # current_velocity = (current_rotmat@current_vel_local.unsqueeze(-1)).squeeze(-1)
        # current_position = torch.as_tensor([current_pose_msg.position.x, current_pose_msg.position.y, current_pose_msg.position.z]).type_as(self.overtaking_curve)

        # tclosest, Pclosest, Vclosest, _ = self.closest_point_finder(
        #     self.overtaking_tstart, self.overtaking_dT,
        #     self.overtaking_curve, current_position)
        # deltas = self.overtaking_psamp - current_position[None]
        # iclosest = torch.argmin(torch.linalg.vector_norm(deltas, dim=-1))
        # tclosest = self.overtaking_tsamp[iclosest]#.item()
        tclosest = (now - self.overtake_start_time).nanoseconds*1e-9
        tsamp = torch.linspace(tclosest, tclosest + 1.6, steps=41).type_as(self.overtaking_curve)

        matrix_factories = {
            self.params.kbezier : self.overall_filter.matrix_factory,
            self.params.kbezier-1 : self.overall_filter.derivative_matrix_factory,
            self.params.kbezier-2 : self.overall_filter.second_derivative_matrix_factory
        }
        numpy_cloud = math_C.to_cavsim_cloud(self.overtaking_curve, self.overtaking_dT, tsamp, matrix_factories,) 

        cloud_msg = ros2_numpy.msgify(sensor_msgs.msg.PointCloud2, numpy_cloud)
        cloud_msg.header.frame_id="map"
        cloud_msg.header.stamp = now.to_msg()
        self.cloud_pub.publish(cloud_msg)


        if tclosest > (self.params.time_horizon):
            self.get_logger().info("Overtaking finished, pausing bag and switching to idle state and setting path tracker back to static raceline")
            self.overtake_end_pub.publish(now.to_msg())
            self.pathswitch_pub.publish(std_msgs.msg.String(data="raceline"))
            stateparam = rclpy.Parameter(DBFOvertakingPathServer.STATE_PARAMETER_NAME, rclpy.Parameter.Type.STRING, "IDLE")
            self.set_parameters([stateparam,])
            # self.pause_service.call_async(rosbag2_interfaces.srv.Pause.Request())
        # else:
        #     self.pathswitch_pub.publish(std_msgs.msg.String(data="graph"))

    def handleStateIdle(self, now : rclpy.time.Time):
        # self.get_logger().info("Overtaking finished, pausing bag and switching to idle state and setting path tracker back to static raceline")
        # pass
        self.overtake_end_pub.publish(now.to_msg())
    def unpause_CB(self, result : rosbag2_interfaces.srv.Resume.Response):
        pass
    def attempt_dbf(self, Curveparticles : torch.Tensor, Curveparticle_tstart : torch.Tensor, Curveparticle_dT : torch.Tensor, rfinal : torch.Tensor, rfinal_min,
                    TV_box_positions : torch.Tensor, TV_stdev_inv_matrix : torch.Tensor):
        
        idx_resample = torch.empty_like(rfinal).long()
        particle_likelihoods = torch.empty_like(rfinal)
        success = torch.as_tensor(0).to(dtype=bool, device=idx_resample.device)
        for i in range(16):
            # minrfinal, maxrfinal = torch.min(rfinal), torch.max(rfinal)
            # self.get_logger().debug("minrfinal: " + str(minrfinal) + " maxrfinal: " + str(maxrfinal) + " rfinal_min: " + str(rfinal_min))
            if i > 0:
                idx_resample_cpu = idx_resample.cpu()
                Curveparticles = Curveparticles[idx_resample_cpu].clone()
                rfinal = rfinal[idx_resample_cpu].clone()
            Curveparticles, rfinal = self.particle_noiser(Curveparticles, Curveparticle_dT, rfinal, rfinal_min)
            (
                (
                    _,
                    _,
                    _,
                    _,
                    _,
                    _,
                    _,
                    _,
                    _,
                    _,
                    _,
                    no_left_bound_violation_probs,
                    no_right_bound_violation_probs,
                ),
                (
                    _,
                    _,
                    _,
                    _,
                    _,
                    overall_collision_free_probs
                ),
                (
                    _,
                    _,
                    _,
                    _,
                    _,
                    _,
                    _,
                    overall_within_limits_probs
                ),
            ) = self.overall_filter(Curveparticles, Curveparticle_tstart, Curveparticle_dT, 
                            TV_box_positions + torch.zeros_like(TV_box_positions), TV_stdev_inv_matrix + torch.zeros_like(TV_stdev_inv_matrix), self.TV_logstdevs + torch.zeros_like(self.TV_logstdevs))
            inbounds_prob = no_left_bound_violation_probs*no_right_bound_violation_probs
            prob_product : torch.Tensor = (overall_within_limits_probs*overall_collision_free_probs*inbounds_prob)
            torch.clip(prob_product.nan_to_num(nan=self.params.min_particle_likelihood, neginf=self.params.min_particle_likelihood, posinf=self.params.min_particle_likelihood),
                        min=self.params.min_particle_likelihood, max=1.0, out=particle_likelihoods)
            torch.gt(torch.max(particle_likelihoods), 0.999, out=success)
            if success.item():
                idx_max = torch.argmax(particle_likelihoods)
                return Curveparticles[idx_max], rfinal[idx_max]
            torch.multinomial(particle_likelihoods, self.params.Nparticles, replacement=True, out=idx_resample)
            if torch.any(idx_resample<0) or torch.any(idx_resample>(idx_resample.shape[0]-1)):
                self.get_logger().error("Invalid indices: " + str(idx_resample))
                return None, None
        return None, None
