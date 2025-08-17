
import deepracing_models.math_utils.convert as math_C
import deepracing_models.math_utils as mu, deepracing_ros.convert as C
from deepracing_ros.controls.path_server_ros import PathServerROS
from deepracing_ros.controls import PlannerParamNames
import numpy as np
import torch
import uva_iac_msgs.msg
import nav_msgs.msg
import std_msgs.msg
import sensor_msgs_py.point_cloud2
import sensor_msgs.msg
import rclpy, rclpy.qos
import scipy.interpolate, scipy.optimize
import time
import threading
from .spliner_utils import SplinerOptim

        # sensor_msgs::PointCloud2Iterator<float> iter_x(*pointcloud, "x");
        # sensor_msgs::PointCloud2Iterator<float> iter_y(*pointcloud, "y");
        # sensor_msgs::PointCloud2Iterator<float> iter_z(*pointcloud, "z");
        # sensor_msgs::PointCloud2Iterator<float> iter_s(*pointcloud, "s");
        # sensor_msgs::PointCloud2Iterator<float> iter_roll(*pointcloud, "roll");
        # sensor_msgs::PointCloud2Iterator<float> iter_heading(*pointcloud, "psi");
        # sensor_msgs::PointCloud2Iterator<float> iter_curvature(*pointcloud, "kappa");
        # sensor_msgs::PointCloud2Iterator<float> iter_vx(*pointcloud, "vx");
        # sensor_msgs::PointCloud2Iterator<float> iter_ax(*pointcloud, "ax");

class SplinerPathServer(PathServerROS):
    def __init__(self):
        super(SplinerPathServer, self).__init__(with_tf=False)
        self.field_names_out=["x", "y", "z", "s", "roll", "psi", "kappa", "vx", "ax"]
        self.pc2fields_out = [
            sensor_msgs.msg.PointField(name=name, offset=4*i, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1)
            for (i,name) in enumerate(self.field_names_out)
        ]
        self.field_names_in=["x", "y", "time", "lat_uncertainty", "lon_uncertainty"]
        #"xt", "yt", "speed", "time",
        self.declare_parameter(PlannerParamNames.STATE, value="CREATED")
        self.declare_parameter(PlannerParamNames.TIMESCALE, value=0.75)
        self.declare_parameter(PlannerParamNames.GPU, value=-1)
        self.declare_parameter(PlannerParamNames.CAR_LENGTH, value=5.2)
        self.declare_parameter(PlannerParamNames.CAR_WIDTH, value=2.0)
        self.newton_iterations = self.declare_parameter("newton_iterations", value=3).get_parameter_value().integer_value
        self.raceline_frenet : mu.RacelineFrenet | None = None
        # self.advantage_factor = self.declare_parameter("advantage_factor", value=1.25).get_parameter_value().double_value

        # self.opponent_odom_msg : nav_msgs.msg.Odometry | None = None
        # self.opponent_odom_mutex = threading.Semaphore()
        # self.opponent_odom_sub  = self.create_subscription(
        #     nav_msgs.msg.Odometry, "target_odom", self.opponentOdomCallback, 10
        # )

        self.opponent_prediction_msg : sensor_msgs.msg.PointCloud2 | None = None
        self.opponent_prediction_mutex = threading.Semaphore()
        predictions_qos = rclpy.qos.QoSProfile(depth=1)
        predictions_qos.reliability = rclpy.qos.ReliabilityPolicy.BEST_EFFORT
        self.opponent_prediction_sub = self.create_subscription(
            sensor_msgs.msg.PointCloud2, "target_predictions", self.opponentPredictionCallback, predictions_qos
        )
        
        self.pc2_pub = self.create_publisher(sensor_msgs.msg.PointCloud2, "pc2_out", predictions_qos)
        self.published_first_path=False
        self.path_switcher = self.create_publisher(std_msgs.msg.String, "path_switch", 1)
        
        self.ego_frenet_pub = self.create_publisher(uva_iac_msgs.msg.FrenetPointStamped, "ego_frenet", 1)
        self.target_frenet_pub = self.create_publisher(uva_iac_msgs.msg.FrenetPointStamped, "target_frenet", 1)

        spliner_params = self.get_parameters_by_prefix("spliner")
        self.optim_wrapper = SplinerOptim(**spliner_params)
        self.initial_guess : np.ndarray | None = None

    def opponentPredictionCallback(self, msg : sensor_msgs.msg.PointCloud2):
        with self.opponent_prediction_mutex:
            self.opponent_prediction_msg = msg

    def getTrajectory(self):
        tick = self.get_clock().now()
        state = self.get_parameter(PlannerParamNames.STATE).get_parameter_value().string_value
        if state != "PLANNING":
            return
        ego_odom = self.getCurrentOdom()
        if ego_odom is None:
            self.get_logger().warn("No ego odometry received yet. Skipping trajectory generation.")
            return
        if self.opponent_prediction_msg is None:
            self.get_logger().warn("No opponent prediction received yet. Skipping trajectory generation.")
            return
        # if self.opponent_odom_msg is None:
        #     self.get_logger().warn("No opponent odometry received yet. Skipping trajectory generation.")
        #     return
        # with self.opponent_odom_mutex:
        #     opponent_odom = deepcopy(self.opponent_odom_msg)
        with self.opponent_prediction_mutex:
            opponent_prediction = sensor_msgs_py.point_cloud2.read_points(self.opponent_prediction_msg, 
                                                                          field_names=self.field_names_in, skip_nans=True)
        car_length : float = self.get_parameter(PlannerParamNames.CAR_LENGTH).get_parameter_value().double_value
        car_width : float = self.get_parameter(PlannerParamNames.CAR_WIDTH).get_parameter_value().double_value
        opponent_positions_np = np.stack([opponent_prediction[k] for k in ["x", "y"]], axis=1)
        opponent_positions = torch.as_tensor(opponent_positions_np).type_as(self.raceline_frenet.rsamp)
        prediction_times = torch.as_tensor(opponent_prediction["time"]).type_as(self.raceline_frenet.rsamp) - opponent_prediction["time"][0].item()
        opponent_lat_uncertainties = torch.as_tensor(opponent_prediction["lat_uncertainty"]).type_as(self.raceline_frenet.rsamp)
        opponent_lon_uncertainties = torch.as_tensor(opponent_prediction["lon_uncertainty"]).type_as(self.raceline_frenet.rsamp)
        opponent_position_spline : scipy.interpolate.BSpline = \
            scipy.interpolate.make_interp_spline(prediction_times.cpu().numpy(), opponent_positions_np, k=2)
        opponent_uncertainty_spline : scipy.interpolate.BSpline = \
            scipy.interpolate.make_interp_spline(prediction_times.cpu().numpy(), torch.stack([opponent_lat_uncertainties, opponent_lon_uncertainties], dim=1).cpu().numpy(), k=1)

        opponent_lat_safety_distances =  3.0*opponent_lat_uncertainties + car_width
        opponent_lon_safety_distances =  3.0*opponent_lon_uncertainties + car_length

        opponent_frenet_s, rl_projections, rl_tangents, _ = self.raceline_frenet.raceline.closest_point_approximate(opponent_positions, newton_iterations=self.newton_iterations)
        rl_tangents : torch.Tensor = rl_tangents/torch.linalg.vector_norm(rl_tangents, dim=-1, keepdim=True)
        rl_normals = rl_tangents[:,[1,0]].clone()
        rl_normals[:,0]*=-1.0
        opponent_frenet_msg = uva_iac_msgs.msg.FrenetPointStamped()
        opponent_frenet_msg.header = ego_odom.header
        opponent_frenet_msg.s = opponent_frenet_s[0].item()
        opponent_frenet_msg.d = torch.sum((opponent_positions[0] - rl_projections[0]) * rl_normals[0]).item()

        ego_position = torch.as_tensor([ego_odom.pose.pose.position.x, ego_odom.pose.pose.position.y]).type_as(self.raceline_frenet.rsamp)
        (ego_current_s,), (projpoint,), (projtangent,), _ = self.raceline_frenet.raceline.closest_point_approximate(ego_position[None], newton_iterations=self.newton_iterations)
        projtangent : torch.Tensor = projtangent/torch.linalg.vector_norm(projtangent)
        projnormal = projtangent[[1,0]].clone()
        projnormal[0]*=-1.0

        (ego_current_t,) = self.raceline_frenet.raceline.t_of_r(ego_current_s[None])
        ego_current_d : float = torch.linalg.vecdot(ego_position - projpoint, projnormal).item()
        ego_frenet_msg = uva_iac_msgs.msg.FrenetPointStamped()
        ego_frenet_msg.header = ego_odom.header
        ego_frenet_msg.s = ego_current_s.item()
        ego_frenet_msg.d = ego_current_d

        self.target_frenet_pub.publish(opponent_frenet_msg)
        self.ego_frenet_pub.publish(ego_frenet_msg)

        


        ego_dense_s = self.raceline_frenet.raceline.__r_of_t__(ego_current_t + prediction_times)[0].squeeze(-1)
        relative_s = ego_dense_s - opponent_frenet_s

        I_cstart = torch.argmax(((relative_s + opponent_lon_safety_distances)>0.0).short())
        t_cstart = prediction_times[I_cstart].item()
        I_cend = torch.argmax(((relative_s - opponent_lon_safety_distances)>0.0).short())
        t_cend = prediction_times[I_cend].item()

        t_spliner = torch.linspace(0.0, prediction_times[-1], steps=30).type_as(prediction_times)
        idx_before_collision = (t_spliner < t_cstart)
        idx_after_collision = (t_spliner > t_cend)
        idx_potential_collision = (~idx_before_collision)*(~idx_after_collision)
        if torch.sum(idx_potential_collision)==0:
            self.get_logger().error("No potential collision detected????")
            return

        potentially_colliding_times = t_spliner[idx_potential_collision].cpu().numpy()
        potentially_colliding_positions = torch.as_tensor(opponent_position_spline(potentially_colliding_times)).type_as(opponent_positions)
        potentially_colliding_uncertainties = torch.as_tensor(opponent_uncertainty_spline(potentially_colliding_times)).type_as(opponent_positions)
        opponent_lat_safety_distances =  3.0*potentially_colliding_uncertainties[:,0] + car_width
        opponent_lon_safety_distances =  3.0*potentially_colliding_uncertainties[:,1] + car_length
        opponent_frenet_s, rlpoints, rlvels, lower_d_opponent, upper_d_opponent = self.raceline_frenet.at_closest_point(potentially_colliding_positions)
        rlspeeds : torch.Tensor = torch.linalg.vector_norm(rlvels, dim=-1)
        rltangents = rlvels/rlspeeds[..., None]
        rlnormals = rltangents[:,[1,0]].clone()
        rlnormals[:,0] *= -1.0
        # opponent_frenet_d : torch.Tensor  = torch.linalg.vecdot(potentially_colliding_positions - rlpoints, rlnormals, dim=-1)
        opponent_frenet_d = 5000.0*torch.ones_like(t_spliner)
        opponent_frenet_d[idx_potential_collision] = torch.linalg.vecdot(potentially_colliding_positions - rlpoints, rlnormals, dim=-1)


        left_overtake_d = opponent_frenet_d[idx_potential_collision] + opponent_lat_safety_distances
        right_overtake_d = opponent_frenet_d[idx_potential_collision] - opponent_lat_safety_distances

        space_on_left = torch.min(upper_d_opponent - left_overtake_d)
        space_on_right = torch.max(lower_d_opponent - right_overtake_d)

        if (space_on_left<0) and (space_on_right>0):
            #If neither side has space. We can't overtake.
            self.get_logger().debug("Overtaking is impossible. insufficient space. Space on left: %f. Space on right: %f" 
                                    % (space_on_left.item(), space_on_right.item()))
            return

        tglobal = t_spliner + ego_current_t
        all_ego_s = self.raceline_frenet.raceline.__r_of_t__(tglobal)[0].squeeze(-1)
        all_ego_d = ego_current_d*torch.ones_like(all_ego_s)
        delta_s = torch.diff(all_ego_s, dim=0)

        _, rlpoints, rlvels, lower_d, upper_d = self.raceline_frenet(all_ego_s)
        rlspeeds : torch.Tensor = torch.linalg.vector_norm(rlvels, dim=-1)
        rltangents = rlvels/rlspeeds[..., None]
        rlnormals = rltangents[:,[1,0]].clone()
        rlnormals[:,0] *= -1.0
        rl2ndderivs = self.raceline_frenet.raceline.__curve_of_r__.__curve_2nd_deriv__(all_ego_s)[0]
        global_kappas : torch.Tensor = torch.linalg.vecdot(rl2ndderivs, rlnormals, dim=-1)
        if self.initial_guess is not None:
            guess = self.initial_guess.copy()
        else:
            guess = all_ego_d.cpu().numpy()
        guess = np.clip(guess, lower_d.cpu().numpy(), upper_d.cpu().numpy())
        guess[0] = ego_current_d
        guess[-2] = guess[-1] = 0.0
        extra_safety_margin = 0.0*opponent_uncertainty_spline(t_spliner.cpu())[:,0]
        result = self.optim_wrapper.optimize_d(
            opponent_frenet_d.cpu().numpy(),  guess, 1.75*car_width*np.ones(opponent_frenet_d.shape[0], dtype=float) + extra_safety_margin,
            lower_d.cpu().numpy(), upper_d.cpu().numpy(), global_kappas.cpu().numpy(), delta_s.cpu().numpy()
        )
        if result.success:
            optimized_ego_d = torch.as_tensor(result.x).type_as(all_ego_s)
            self.initial_guess = optimized_ego_d.cpu().numpy()
        else:
            self.get_logger().debug("Optimization failed: "  + result.message)
            return

        dv_dr : torch.Tensor = self.raceline_frenet.raceline.__dspeed_dr__(all_ego_s)[0].squeeze(-1)
        rlaccels = dv_dr * rlspeeds
        # rlaccels = self.raceline_frenet.raceline.__along_of_t__(tglobal)[0].squeeze(-1)
        t_spliner_cpu = t_spliner.cpu()
        overtaking_points = rlpoints + rlnormals*optimized_ego_d[:,None]
        splineout : scipy.interpolate.BSpline = scipy.interpolate.make_interp_spline(
            t_spliner_cpu, overtaking_points.cpu(), k=3
        )
        velout : np.ndarray = splineout(t_spliner_cpu, nu=1)
        speedsout : np.ndarray = np.linalg.norm(velout, ord=2.0, axis=-1, keepdims=False)
        tangentsout = velout/speedsout[:,None]
        normalsout = tangentsout[:,[1,0]].copy()
        normalsout[:,0] *= -1.0
        accelout = splineout(t_spliner_cpu, nu=2)
        lateral_accelout = np.abs(np.sum(accelout*normalsout, axis=1))
        kappa = lateral_accelout/np.square(speedsout)
        
        # self.field_names_out=["x", "y", "z", "s", "roll", "psi", "kappa", "vx", "ax"]
        maxkappa = np.max(kappa)
        maxlateral_accel = np.max(lateral_accelout)
        self.get_logger().debug("Max Speed: %f. Max Kappa: %f. Max Lat Accel: %f. Max Long Accel: %f" % (rlspeeds.max().item(), float(maxkappa), float(maxlateral_accel), rlaccels.max().item()))
        points_out = np.zeros([rlpoints.shape[0], len(self.field_names_out)], dtype=np.float32)
        #x,y. 
        points_out[:,:2] = overtaking_points.cpu().float()
        
        #leave z as zero

        #arclengths
        points_out[1:,3] = torch.cumsum(torch.linalg.vector_norm(overtaking_points[1:] - overtaking_points[:-1], dim=-1), 0).cpu().float()

        #leave roll as 0

        #psi
        points_out[:,5] = np.arctan2(tangentsout[:,1], tangentsout[:,0])
        #kappa
        points_out[:,6] = kappa
        #vels
        points_out[:,7] = (rlspeeds).cpu().numpy()
        #accels
        points_out[:,8] = (rlaccels).cpu().numpy()
        tock = self.get_clock().now()

        delta = tock - tick

        delta_seconds = float(delta.nanoseconds) * 1E-9
        if delta_seconds>0.2:
            self.get_logger().error("Took too long, not publishing")
            return
        
        cloud_msg = sensor_msgs_py.point_cloud2.create_cloud(ego_odom.header, self.pc2fields_out, points_out.tolist())
        self.pc2_pub.publish(cloud_msg)
        if not self.published_first_path:
            self.path_switcher.publish(std_msgs.msg.String(data="graph"))
            self.published_first_path = True



    def initialize(self, raceline_structured : np.ndarray, innerbound_structured : np.ndarray, outerbound_structured : np.ndarray):
        self.set_parameters([rclpy.Parameter(PlannerParamNames.STATE, rclpy.Parameter.Type.STRING, "INITIALIZING"),])
        gpu = self.get_parameter(PlannerParamNames.GPU).get_parameter_value().integer_value
        device = torch.device("cuda:%d" % gpu if gpu>=0 else "cpu")
        timescale = self.get_parameter(PlannerParamNames.TIMESCALE).get_parameter_value().double_value
        torch.set_float32_matmul_precision("high")
        times_in = raceline_structured["time"].astype(np.float64)
        line_all_points = torch.as_tensor(np.stack([raceline_structured[k] for k in ["x", "y"]], axis=1), dtype=torch.float32, device=device)
        # interp_spline : scipy.interpolate.BSpline = \
        #     scipy.interpolate.make_interp_spline(times_in, line_all_points.cpu().numpy(), k=2, bc_type="periodic")
        # interp_times = np.linspace(times_in[0], times_in[-1], num=int(round(times_in[-1].item()/0.075)))
        # interp_spline_points = interp_spline(interp_times)
        # interp_spline_speeds = np.linalg.norm(interp_spline(interp_times, nu=1), ord=2.0, axis=1)
        # line_all_speeds = torch.as_tensor(interp_spline_speeds).double()
        car_length = self.get_parameter(PlannerParamNames.CAR_LENGTH).get_parameter_value().double_value
        drsamp = 0.6*car_length
        line_all_speeds = torch.as_tensor(raceline_structured["speed"]).type_as(line_all_points)
        self.get_logger().info("Building Raceline Helper")
        _raceline_helper_ : mu.RacelineHelper = mu.RacelineHelper.from_closed_path(
            line_all_points, timescale*line_all_speeds,
            drsamp
        ).to(tensor=line_all_points)

        self.get_logger().info("Building Inner Boundary Helper")
        innerbound = torch.as_tensor(np.stack([innerbound_structured[k] for k in ["x", "y"]], axis=1)).type_as(line_all_points)
        arbitrary_s = np.linspace(0.0, float(innerbound.shape[0]), num=innerbound.shape[0])
        innerbound_spline = scipy.interpolate.make_interp_spline(arbitrary_s, innerbound.cpu().numpy(), k=3, bc_type="periodic")
        innerbound_tangents = innerbound_spline(arbitrary_s, nu=1)
        innerbound_tangents = innerbound_tangents/np.linalg.norm(innerbound_tangents, ord=2.0, axis=1, keepdims=True)
        innerbound_normals = torch.as_tensor(np.stack([-innerbound_tangents[:, 1], innerbound_tangents[:, 0]], axis=1)).type_as(line_all_points)
        _innerbound_helper_ : mu.SimplePathHelper = mu.SimplePathHelper.from_closed_path(innerbound - 0.5*innerbound_normals, drsamp)

        self.get_logger().info("Building Outer Boundary Helper")
        outerbound = torch.as_tensor(np.stack([outerbound_structured[k] for k in ["x", "y"]], axis=1)).type_as(line_all_points)
        arbitrary_s = np.linspace(0.0, float(outerbound.shape[0]), num=outerbound.shape[0])
        outerbound_spline = scipy.interpolate.make_interp_spline(arbitrary_s, outerbound.cpu().numpy(), k=3, bc_type="periodic")
        outerbound_tangents = outerbound_spline(arbitrary_s, nu=1)
        outerbound_tangents = outerbound_tangents/np.linalg.norm(outerbound_tangents, ord=2.0, axis=1, keepdims=True)
        outerbound_normals = torch.as_tensor(np.stack([-outerbound_tangents[:, 1], outerbound_tangents[:, 0]], axis=1)).type_as(line_all_points)
        _outerbound_helper_ : mu.SimplePathHelper = mu.SimplePathHelper.from_closed_path(outerbound + 0.5*outerbound_normals, drsamp)

        self.get_logger().info("Building Raceline Frenet")
        _raceline_frenet_ = mu.RacelineFrenet(_raceline_helper_, _innerbound_helper_, _outerbound_helper_, drsamp).to(tensor=line_all_points)
        backend = "inductor"
        mode="max-autotune-no-cudagraphs"
        torch.set_float32_matmul_precision('high')
        
        self.get_logger().info("Compiling Raceline Frenet")
        self.raceline_frenet : mu.RacelineFrenet = torch.compile(
            _raceline_frenet_,  backend=backend, mode=mode, dynamic=True, fullgraph=True
        )
        comptimes = []
        for asdf in range(16):
            tick = time.time()
            #compile and warmup the path helpers
            tstart = torch.rand(1, dtype=torch.float32).item()*(_raceline_helper_.__r_of_t__.xend_vec[-1].item())
            tend = tstart + 7.0
            tsamp = torch.linspace(tstart, tend, steps=30).type_as(line_all_points)
            tsamp_dense = torch.linspace(tstart, tend, steps=200).type_as(line_all_points)
            # rsamp, _ = self.raceline_frenet.raceline.__r_of_t__(tsamp)
            rsamp, rlpoints, rlvels, _ = self.raceline_frenet.raceline(t=tsamp)
            self.raceline_frenet.raceline(r=rsamp)
            self.raceline_frenet.raceline.__dspeed_dr__(rsamp)
            rlpoints_noisy = rlpoints + 2.0*torch.randn_like(rlpoints)

            rtrue, rlpointsback, rlvelsback, ib_widths, ob_widths = self.raceline_frenet.at_closest_point(rlpoints_noisy, newton_iterations=self.newton_iterations)
            self.raceline_frenet(rsamp)
            self.raceline_frenet(rtrue)
            rtrue, rlpointsback, rlvelsback, ib_widths, ob_widths = self.raceline_frenet.at_closest_point(rlpoints_noisy[[0,]], newton_iterations=self.newton_iterations)
            self.raceline_frenet(rsamp)
            self.raceline_frenet(rtrue)
            self.raceline_frenet.raceline.closest_point_approximate(rlpoints_noisy[[0,]], newton_iterations=self.newton_iterations)

            rsamp_dense, rlpoints_dense, rlvels_dense, _ = self.raceline_frenet.raceline(t=tsamp_dense)
            self.raceline_frenet(rsamp_dense)
            self.raceline_frenet.raceline(r=rsamp_dense)
            self.raceline_frenet.raceline.__dspeed_dr__(rsamp_dense)
            tock = time.time()
            comptimes.append(tock-tick)
        comptimes = 1000.0*torch.as_tensor(comptimes, dtype=torch.float64)
        self.get_logger().info("comptimes: " + str(comptimes))
        self.set_parameters([rclpy.Parameter(PlannerParamNames.STATE, rclpy.Parameter.Type.STRING, "PLANNING"),])

            
