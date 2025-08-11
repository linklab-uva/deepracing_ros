
from copy import deepcopy
import deepracing_models.math_utils.convert as math_C
import deepracing_models.math_utils as mu, deepracing_ros.convert as C
from deepracing_ros.controls.path_server_ros import PathServerROS
from deepracing_ros.controls import PlannerParamNames
import numpy as np
import torch
import nav_msgs.msg
import sensor_msgs_py.point_cloud2
import sensor_msgs.msg
import rclpy, rclpy.qos
import scipy.interpolate, scipy.optimize
import time
import threading


class SplinerPathServer(PathServerROS):
    def __init__(self):
        super(SplinerPathServer, self).__init__()
        self.field_names=["x", "y",  "lat_uncertainty", "lon_uncertainty"]
        #"xt", "yt", "speed", "time",
        self.declare_parameter(PlannerParamNames.STATE, value="CREATED")
        self.declare_parameter(PlannerParamNames.TIMESCALE, value=0.75)
        self.declare_parameter(PlannerParamNames.GPU, value=-1)
        self.raceline_frenet : mu.RacelineFrenet | None = None

        self.opponent_odom_msg : nav_msgs.msg.Odometry | None = None
        self.opponent_odom_mutex = threading.Semaphore()
        self.opponent_odom_sub  = self.create_subscription(
            nav_msgs.msg.Odometry, "opponent_odom", self.opponentOdomCallback, 10
        )

        self.opponent_prediction_msg : sensor_msgs.msg.PointCloud2 | None = None
        self.opponent_prediction_mutex = threading.Semaphore()
        predictions_qos = rclpy.qos.QoSProfile(depth=1)
        predictions_qos.reliability = rclpy.qos.ReliabilityPolicy.BEST_EFFORT
        self.opponent_prediction_sub = self.create_subscription(
            sensor_msgs.msg.PointCloud2, "opponent_predictions", self.opponentPredictionCallback, predictions_qos
        )

        
    def opponentOdomCallback(self, msg : nav_msgs.msg.Odometry):
        with self.opponent_odom_mutex:
            self.opponent_odom_msg = msg

    def opponentPredictionCallback(self, msg : sensor_msgs.msg.PointCloud2):
        with self.opponent_prediction_mutex:
            self.opponent_prediction_msg = msg

    def getTrajectory(self):
        state = self.get_parameter(PlannerParamNames.STATE).get_parameter_value().string_value
        if state != "PLANNING":
            return
        if self.opponent_odom_msg is None:
            self.get_logger().warn("No opponent odometry received yet. Skipping trajectory generation.")
            return
        if self.opponent_prediction_msg is None:
            self.get_logger().warn("No opponent prediction received yet. Skipping trajectory generation.")
            return
        with self.opponent_odom_mutex:
            opponent_odom = deepcopy(self.opponent_odom_msg)
        with self.opponent_prediction_mutex:
            opponent_prediction = sensor_msgs_py.point_cloud2.read_points(self.opponent_prediction_msg, 
                                                                          field_names=self.field_names, skip_nans=True)
        opponent_positions = torch.as_tensor(np.stack([opponent_prediction[k] for k in ["x", "y"]], axis=1)).type_as(self.raceline_frenet.rsamp)
        opponent_lat_uncertainties = torch.as_tensor(opponent_prediction["lat_uncertainty"]).type_as(self.raceline_frenet.rsamp)
        opponent_lon_uncertainties = torch.as_tensor(opponent_prediction["lon_uncertainty"]).type_as(self.raceline_frenet.rsamp)

        opponent_frenet_s, rlpoints, rlvels, ib_widths, ob_widths = self.raceline_frenet.at_closest_point(opponent_positions)
        rlspeeds : torch.Tensor = torch.linalg.vector_norm(rlvels, dim=-1)
        rltangents = rlvels/rlspeeds[..., None]
        rlnormals = rltangents[:,[1,0]].clone()
        rlnormals[:,0] *= -1.0
        opponent_frenet_d : torch.Tensor  = torch.linalg.vecdot(opponent_positions - rlpoints, rlnormals, dim=-1)
        

        



    def initialize(self, raceline_structured : np.ndarray, innerbound_structured : np.ndarray, outerbound_structured : np.ndarray):
        self.set_parameters([rclpy.Parameter(PlannerParamNames.STATE, rclpy.Parameter.Type.STRING, "INITIALIZING"),])
        gpu = self.get_parameter(PlannerParamNames.GPU).get_parameter_value().integer_value
        device = torch.device("cuda:%d" % gpu if gpu>=0 else "cpu")
        timescale = self.get_parameter(PlannerParamNames.TIMESCALE).get_parameter_value().double_value
        torch.set_float32_matmul_precision("high")
        times_in = raceline_structured["time"].astype(np.float64)
        line_all_points = torch.as_tensor(np.stack([raceline_structured[k] for k in ["x", "y"]], axis=1), dtype=torch.float32, device=device)
        interp_spline : scipy.interpolate.BSpline = \
            scipy.interpolate.make_interp_spline(times_in, line_all_points.cpu().numpy(), k=2, bc_type="periodic")
        interp_times = np.linspace(times_in[0], times_in[-1], num=int(round(times_in[-1].item()/0.075)))
        interp_spline_points = interp_spline(interp_times)
        interp_spline_speeds = np.linalg.norm(interp_spline(interp_times, nu=1), ord=2.0, axis=1)
        line_all_speeds = torch.as_tensor(interp_spline_speeds).double()
        self.get_logger().info("Building Raceline Helper")
        _raceline_helper_ : mu.RacelineHelper = mu.RacelineHelper.from_closed_path(
            torch.as_tensor(interp_spline_points).type_as(line_all_speeds), timescale*line_all_speeds,
            0.5
        ).to(tensor=line_all_points)
        drsamp = 0.5

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
            # rsamp, _ = self.raceline_frenet.raceline.__r_of_t__(tsamp)
            rsamp, rlpoints, rlvels, _ = self.raceline_frenet.raceline(t=tsamp)
            self.raceline_frenet.raceline(r=rsamp)
            
            rlpoints_noisy = rlpoints + 2.0*torch.randn_like(rlpoints)

            rtrue, rlpointsback, rlvelsback, ib_widths, ob_widths = self.raceline_frenet.at_closest_point(rlpoints_noisy)
            self.raceline_frenet(rsamp)
            self.raceline_frenet(rtrue)
            tock = time.time()
            comptimes.append(tock-tick)
        comptimes = 1000.0*torch.as_tensor(comptimes, dtype=torch.float64)
        self.get_logger().info("comptimes: " + str(comptimes))
        self.set_parameters([rclpy.Parameter(PlannerParamNames.STATE, rclpy.Parameter.Type.STRING, "PLANNING"),])

            
