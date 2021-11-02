#control points that define the bezier curve
geometry_msgs/Point[] control_points
#list of row-major 3X3 covariances for each control point.
CovariancePoint[] control_point_covariances
#control_point_covariances and control_points should either have the same length
#or control_point_covariances should be empty to indicate no covariance information is available
#Header containing a timestamp and the frame_id of all control points
std_msgs/Header header
#How long of a timestamp the curve represents
builtin_interfaces/Duration delta_t 