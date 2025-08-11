
import deepracing_models.math_utils.convert as math_C
import deepracing_models.math_utils as mu, deepracing_ros.convert as C
from deepracing_ros.controls.path_server_ros import PathServerROS
from deepracing_ros.controls import PlannerParamNames
import numpy as np
import torch
import nav_msgs.msg
import sensor_msgs_py.point_cloud2
import sensor_msgs.msg
import rclpy, rclpy.parameter
import scipy.interpolate, scipy.optimize
import time
import threading

class SplinerPathServer(PathServerROS):
    def __init__(self):
        super(SplinerPathServer, self).__init__()
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
        self.opponent_prediction_sub = self.create_subscription(
            sensor_msgs.msg.PointCloud2, "opponent_prediction", self.opponentPredictionCallback, 10
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
        # self.get_logger().info("Getting trajectory")


    def initialize(self, raceline_structured : np.ndarray, innerbound_structured : np.ndarray, outerbound_structured : np.ndarray):
        self.set_parameters([rclpy.Parameter(PlannerParamNames.STATE, rclpy.Parameter.Type.STRING, "INITIALIZING"),])
        gpu = self.get_parameter(PlannerParamNames.GPU).get_parameter_value().integer_value
        device = torch.device("cuda:%d" % gpu if gpu>=0 else "cpu")
        timescale = self.get_parameter(PlannerParamNames.TIMESCALE).get_parameter_value().double_value
        torch.set_float32_matmul_precision("high")
        times_in = raceline_structured["time"].astype(np.float64)
        line_all_points = torch.as_tensor(np.stack([raceline_structured["x"], raceline_structured["y"]], axis=1), dtype=torch.float32, device=device)
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
        _innerbound_helper_ : mu.SimplePathHelper = mu.SimplePathHelper.from_closed_path(innerbound, drsamp)

        self.get_logger().info("Building Outer Boundary Helper")
        outerbound = torch.as_tensor(np.stack([outerbound_structured[k] for k in ["x", "y"]], axis=1)).type_as(line_all_points)
        _outerbound_helper_ : mu.SimplePathHelper = mu.SimplePathHelper.from_closed_path(outerbound, drsamp)

        self.get_logger().info("Building Raceline Frenet")
        _raceline_frenet_ = mu.RacelineFrenet(_raceline_helper_, _innerbound_helper_, _outerbound_helper_, drsamp)
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

            rlpointsback, ib_widths, ob_widths = self.raceline_frenet.at_closest_point(rlpoints_noisy)
            self.raceline_frenet(rsamp)
            tock = time.time()
            comptimes.append(tock-tick)
        comptimes = 1000.0*torch.as_tensor(comptimes, dtype=torch.float64)
        self.get_logger().info("comptimes: " + str(comptimes))
        self.set_parameters([rclpy.Parameter(PlannerParamNames.STATE, rclpy.Parameter.Type.STRING, "PLANNING"),])

            
