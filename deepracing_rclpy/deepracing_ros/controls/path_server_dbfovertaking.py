import time
from copy import deepcopy
import numpy as np
import torch
from scipy.spatial.transform import Rotation
from deepracing_msgs.msg import CompositeBezierCurve
import rclpy
import rclpy.qos
import rclpy.publisher
import rclpy.subscription

import deepracing_models.math_utils as mu, deepracing_ros.convert as C
from deepracing_models.math_utils.bayesian_filtering import BayesianFilter, ParticleNoiser
from deepracing_models.math_utils.bounds_checking import BoundsChecker
from deepracing_models.math_utils.statistics import CollisionProbabilityEstimator
from deepracing_models.math_utils.dynamics import ExceedLimitsProbabilityEstimator
from deepracing_ros.controls.path_server_ros import PathServerROS
from deepracing_rclpy.dbf_overtaking import dbf_overtaking
import scipy.interpolate
import threading

class DBFOvertakingPathServer(PathServerROS):
    def __init__(self):
        super(DBFOvertakingPathServer, self).__init__()
        self.get_logger().info("Hello Path Server! I live in namespace: %s" % (self.get_namespace()))

        
        self.param_listener = dbf_overtaking.ParamListener(self)
        self.params = self.param_listener.get_params()
        
        self.opponent_curve_mutex = threading.Semaphore()
        self.opponent_curve_msg : CompositeBezierCurve | None = None
        self.opponent_bcurve_sub : rclpy.subscription.Subscription = self.create_subscription(CompositeBezierCurve, "opponent_curve", self.opponent_curve_CB, rclpy.qos.qos_profile_sensor_data)
        self.composite_bcurve_pub : rclpy.publisher.Publisher = None #self.create_publisher(CompositeBezierCurve, "oraclecompositebeziercurves", 1)

    def opponent_curve_CB(self, msg : CompositeBezierCurve):
        if not self.opponent_curve_mutex.acquire(timeout=0.1):
            raise ValueError("Unable to acquire opponent_curve_mutex")
        self.opponent_curve_msg = msg
        self.opponent_curve_mutex.release()

    def initialize(self, raceline_structured : np.ndarray, widthmap_structured : np.ndarray, innerbound_structured : np.ndarray, outerbound_structured : np.ndarray):
        
        device = torch.device("cuda:%d" % self.params.gpu if self.params.gpu>=0 else "cpu")
        self.get_logger().info("Building Raceline Helper")
        line_all_points = torch.as_tensor(np.stack([raceline_structured["x"], raceline_structured["y"]], axis=1), dtype=torch.float32, device=device)
        times_in = raceline_structured["time"].astype(np.float64)
        interp_spline : scipy.interpolate.BSpline = \
            scipy.interpolate.make_interp_spline(times_in, line_all_points.cpu().numpy(), k=3, bc_type="periodic")
        interp_spline_speeds = np.linalg.norm(interp_spline(times_in, nu=1), ord=2.0, axis=1)
        line_all_speeds = self.params.raceline_scale*torch.as_tensor(interp_spline_speeds).double()
        _raceline_helper_ : mu.RacelineHelper = mu.RacelineHelper.from_closed_path(
            line_all_points.cpu().double(), line_all_speeds,
            0.5
        ).to(tensor=line_all_points)
        self.get_logger().info("Built Raceline Helper")
        self.get_logger().info("Building Bounds Checker")
        shrink_factor = 1.0
        centerline_dense = torch.as_tensor(np.stack([widthmap_structured[k] for k in ["x", "y"]], axis=1)).type_as(line_all_speeds)
        left_widths = shrink_factor*torch.as_tensor(widthmap_structured["ob_distance"]).type_as(line_all_speeds) + 0.1*self.params.car_dims.width
        right_widths = shrink_factor*torch.as_tensor(widthmap_structured["ib_distance"]).type_as(line_all_speeds) - 0.1*self.params.car_dims.width
        _bounds_checker_ = BoundsChecker(gauss_order=self.params.bounds_gauss.order, dT=self.params.time_horizon, stdev=self.params.bounds_gauss.stdev,
            dr_samp=self.params.bounds_gauss.dr_samp, alpha=self.params.bounds_gauss.alpha,
            left_widths=left_widths, right_widths=right_widths, refline_points=centerline_dense,
            newton_iterations=self.params.bounds_newton.iterations, newton_stepsize=self.params.bounds_newton.stepsize, max_step=self.params.bounds_newton.max_step
        ).eval().to(tensor=line_all_points)
        self.get_logger().info("Built Bounds Checker")

        self.get_logger().info("Building Dynamics Checker")
        brake_factor = long_accel_factor = lat_accel_factor = 1.0
        _dynamic_violation_estimator_ = ExceedLimitsProbabilityEstimator(
            (1.0 + 0.000)*torch.as_tensor([-1.0,    0.00,    25.190,  40.192,  64.544,  75.197,  89.330,  1000.0]), 
            brake_factor*torch.as_tensor( [-14.574,  -14.574, -14.574, -17.701, -21.424, -23.359, -25.593, -25.593]),

            (1.0 + 0.000)*torch.as_tensor(    [-1.0,    0.0,    24.102,  40.192,  48.237,  59.325, 75.850, 91.069, 92.5,  1000.0]), 
            long_accel_factor*torch.as_tensor([ 14.162,   14.162, 14.162,  12.971,  12.375,  9.546,  4.484,  0.464,  0.0,   0.0]),

            (1.0 + 0.000)*torch.as_tensor(   [-1.00,   0.0,    19.0,   75.850,  91.069,  1000.0]), 
            lat_accel_factor*torch.as_tensor([ 12.224,   12.224, 16.224, 35.156,  40.218,  40.218]),
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
            self.params.collision_gauss.order_time, self.params.time_horizon, self.params.collision_gauss.order_space, lat_buffer, long_buffer, alpha=self.params.collision_gauss.alpha
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
        rfinal_min = (rtarget[0,-1] + 1.5*car_length)
        self.get_logger().info("Built Overall Filter")
        compile_backend = self.params.compile_backend
        compile_modules = len(compile_backend)>0
        backend = compile_backend if compile_modules else "inductor"
        mode="max-autotune-no-cudagraphs" if backend=="inductor" else None
        self.get_logger().info("Compiling Overall Filter")
        # torch.set_float32_matmul_precision('high')
        self.raceline_helper : mu.RacelineHelper = torch.compile(_raceline_helper_, fullgraph=True, dynamic=True, mode=mode, backend=backend, disable=(not compile_modules))
        # self.t_of_r : mu.TofRHelper = torch.compile(mu.TofRHelper(_raceline_helper_.__r_of_t__.control_points.detach().clone(), _raceline_helper_.__times_in__.detach().clone()), fullgraph=True, dynamic=True, mode=mode, backend=backend, disable=(not compile_modules))
        self.particle_noiser : ParticleNoiser = torch.compile(_particle_noiser_, fullgraph=True, dynamic=False, mode=mode, backend=backend, disable=(not compile_modules))
        self.overall_filter : BayesianFilter = torch.compile(_overall_filter_, fullgraph=True, dynamic=False, mode=mode, backend=backend, disable=(not compile_modules))
        warmup_runs = 20
        warmup_times = []
        tfit = torch.linspace(0.0, self.params.time_horizon, steps=60).type_as(Curveparticles)
        for i in range(warmup_runs):
            rfinal_ = rfinal + 50.0*torch.randn_like(rfinal)
            TV_box_positions_ = TV_box_positions + 75.0*torch.randn_like(TV_box_positions)
            tick = time.time()
            Curveparticles_ = Curveparticles + 75.0*torch.randn_like(Curveparticles)#Curveparticle_tstart + 0.025*torch.randn_like(Curveparticle_tstart)
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
            tock = time.time()
            warmup_times.append(tock-tick)
        self.tfit = tfit
        self.Curveparticles = None
        self.Curveparticle_tstart = Curveparticle_tstart
        self.Curveparticle_dT = Curveparticle_dT
        warmup_times = torch.as_tensor(warmup_times, dtype=torch.float64)
        self.get_logger().info("Compiled Overall Filter")
        self.get_logger().info("warmup_times: " + str(warmup_times))
        self.composite_bcurve_pub : rclpy.publisher.Publisher = self.create_publisher(CompositeBezierCurve, "bcurvesout", 1)
    
    def getTrajectory(self):
        if self.current_odom is None:
            self.get_logger().error("No odom yet")
            return
        # current_pose_msg = deepcopy(self.current_odom.pose.pose)
        # current_vel_msg = deepcopy(self.current_odom.twist.twist)
        # current_rot = Rotation.from_quat([0.0, 0.0, current_pose_msg.orientation.z, current_pose_msg.orientation.w])
        # current_rotmat = torch.as_tensor(current_rot.as_matrix()[0:2,0:2]).type_as(self.Curveparticle_tstart)
        # current_position_msg = current_pose_msg.position
        # current_position = torch.as_tensor([current_position_msg.x, current_position_msg.y]).type_as(self.Curveparticle_tstart)
        # current_velocity = (current_rotmat@torch.as_tensor([[current_vel_msg.linear.x,], [current_vel_msg.linear.y,]]).type_as(self.Curveparticle_tstart))[:,0]
        # rclosest, _, _, _ = self.raceline_helper.closest_point_approximate(current_position[None], newton_iterations=3)
        # tclosest = self.raceline_helper.t_of_r(rclosest).item()
        # if self.Curveparticles is None:
            # tfit = torch.linspace(tclosest, tclosest + self.params.time_horizon, steps=40).type_as(self.Curveparticle_tstart)
        # rfit, pfit, vfit, _ = self.raceline_helper(t=self.tfit + tclosest)
        # Curveparticles, _ = mu.compositeBezierFit(self.tfit[None], pfit[None], numsegments=self.params.Nsegments, kbezier=self.params.kbezier,
        #                                             Y_0=current_position[None], dYdT_0=vfit[[0,],], 
        #                                             Y_f=pfit[[-1,]], dYdT_f=vfit[[-1],],
        #                                             constraint_level=2 )
        # rfinal = rfit[[-1,]].expand(self.params.Nparticles).clone()
        # self.Curveparticles = Curveparticles.expand(self.params.Nparticles, *Curveparticles.shape[1:]).clone()
        if self.opponent_curve_msg is not None:
            if not self.opponent_curve_mutex.acquire(timeout=0.1):
                raise ValueError("Unable to acquire opponent_curve_mutex")
            TV_curve_dT, Targetvehicle_curve  = C.fromCompositeBezierCurveMsg(self.opponent_curve_msg, dtype=self.tfit.dtype, device=self.tfit.device)
            self.opponent_curve_mutex.release()
            Targetvehicle_curve = Targetvehicle_curve[...,[0,1]]
            # print(Targetvehicle_curve.shape)
            # print(TV_curve_dT.shape)
            TV_curve_tstart = torch.cumsum(TV_curve_dT, 0)-TV_curve_dT[0]
            # print(TV_curve_tstart.shape)
            Targetvehicle_curve_deriv = self.params.kbezier*torch.diff(Targetvehicle_curve, dim=-2)/TV_curve_dT[:, None, None]
            tnodes = self.overall_filter.collision_probability_estimator.gl1d.eta
            # print(tnodes.shape)
            TV_positions, _ = mu.compositeBezierEval(TV_curve_tstart, TV_curve_dT, Targetvehicle_curve, tnodes, self.overall_filter.matrix_factory) 
            TV_rfinal, _, _, _ = self.raceline_helper.closest_point_approximate(Targetvehicle_curve[-1,[-1,]], newton_iterations=3)
            TV_rinitial, _, _, _ = self.raceline_helper.closest_point_approximate(Targetvehicle_curve[0,[0,]], newton_iterations=3)
            TV_tinitial = self.raceline_helper.t_of_r(TV_rinitial)
            rfit, pfit, vfit, _ = self.raceline_helper(t=self.tfit + TV_tinitial.item() - 0.5)
            Curveparticles, _ = mu.compositeBezierFit(self.tfit[None], pfit[None], numsegments=self.params.Nsegments, kbezier=self.params.kbezier,
                                                        Y_0=pfit[[0,]], dYdT_0=vfit[[0,],], 
                                                        Y_f=pfit[[-1,]], dYdT_f=vfit[[-1],],
                                                        constraint_level=2 )
            rfinal = rfit[[-1,]].expand(self.params.Nparticles).clone()
            self.Curveparticles = Curveparticles.expand(self.params.Nparticles, *Curveparticles.shape[1:]).clone()
            rfinal_min = TV_rfinal[0] + 2.0*self.params.car_dims.length
            TV_velocities, _ = mu.compositeBezierEval(TV_curve_tstart, TV_curve_dT, Targetvehicle_curve_deriv, tnodes, self.overall_filter.derivative_matrix_factory) 
            TV_tangents : torch.Tensor = TV_velocities/torch.linalg.vector_norm(TV_velocities, dim=-1, keepdim=True)
            TV_normals = TV_tangents[:,[1,0]].clone()
            TV_normals[:,0]*=-1.0
            TV_rotmats = torch.stack([TV_tangents, TV_normals], dim=-1)
            TV_box_positions = (TV_rotmats@self.boxpoints_target_01.transpose(-2,-1)).transpose(-2,-1) + TV_positions[:,None]
            
            TV_eigvecs = TV_rotmats[None,:,None].expand(self.params.Nparticles, TV_positions.shape[0], self.boxpoints_target_01.shape[0], 2, 2)#.clone()
            TV_stdev_inv_matrix : torch.Tensor = self.TV_invdiag@TV_eigvecs.transpose(-2,-1)
            TV_box_positions = TV_box_positions[None].expand(self.params.Nparticles, *TV_box_positions.shape)
            # print(rfinal[0])
            # print(rfinal_min)
            tick = time.time()
            dbf_curve = self.attempt_dbf(
                self.Curveparticles, self.Curveparticle_tstart, self.Curveparticle_dT, rfinal, rfinal_min,
                    TV_box_positions, TV_stdev_inv_matrix
            )
            tock = time.time()
            if dbf_curve is not None:
                msgout = C.toCompositeBezierCurveMsg(self.Curveparticle_dT[0], dbf_curve)
                msgout.header.frame_id="map"
                self.get_logger().info("YAY! DBF algorithm converged in %f seconds" % (tock-tick,))
                self.composite_bcurve_pub.publish(msgout)
            else:
                self.get_logger().error("DBF algorithm did not converge in %f seconds" % (tock-tick,))

    def attempt_dbf(self, Curveparticles : torch.Tensor, Curveparticle_tstart : torch.Tensor, Curveparticle_dT : torch.Tensor, rfinal : torch.Tensor, rfinal_min,
                    TV_box_positions : torch.Tensor, TV_stdev_inv_matrix : torch.Tensor):
        
        for i in range(16):
            idx_resample = torch.arange(0, Curveparticles.shape[0], step=1, dtype=torch.int64, device=Curveparticles.device)
            particle_likelihoods : torch.Tensor = torch.ones(idx_resample.shape[0]).type_as(Curveparticles)/self.params.Nparticles
            success = (particle_likelihoods[0]<0.0)
            Curveparticles[:] = Curveparticles[idx_resample]#.clone()\
            rfinal[:] = rfinal[idx_resample]
            Curveparticles, rfinal = self.particle_noiser(Curveparticles, Curveparticle_dT, rfinal, rfinal_min)

            (
                (
                    closest_point_r,
                    bounds_check_positions,
                    closest_point_values,
                    closest_point_tangents,
                    closest_point_normals,
                    deltas,
                    signed_distances,
                    left_width_vals,
                    right_width_vals,
                    specific_left_bound_violation_probs,
                    specific_right_bound_violation_probs,
                    no_left_bound_violation_probs,
                    no_right_bound_violation_probs,
                ),
                (
                    gauss_pts,
                    gaussian_pdf_vals,
                    collision_check_positions,
                    collision_probs,
                    overall_lambdas,
                    overall_collision_free_probs
                ),
                (
                    ellipse_points,
                    ellipse_normals,
                    origin,
                    lat_radii,
                    long_radii,
                    signed_distances,
                    specific_dynamic_violation_probs,
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
                return Curveparticles[idx_max]
            torch.multinomial(particle_likelihoods, self.params.Nparticles, replacement=True)
        return None
