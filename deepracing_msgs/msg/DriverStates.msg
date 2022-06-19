#Timestamp and frame for the geometric information
std_msgs/Header header

#vehicle indices that are actually producing valid data. Can be used to index the other data arrays from the various deepracing message types
uint8[<=19] vehicle_indices
#Which sector the other agents are in
uint8[<=19] other_agent_sectors
#Pose information for the other agents on the track (could be empty in the case of a time trial)
geometry_msgs/Pose[<=19] other_agent_poses
#Linear velocity information for the other agents on the track (could be empty in the case of a time trial)
geometry_msgs/Vector3[<=19] other_agent_velocities
#Linear projection distances along the centerline of the track, negative implies behind starting line, positive implies forward past the starting line
float64[<=19] other_agent_track_progress

#vehicle index for the ego vehicle
uint8 ego_vehicle_index
#Which sector the ego vehicle is in.
uint8 ego_current_sector
#Pose information for the ego vehicle. 
geometry_msgs/Pose ego_pose
#Linear & Angular velocity information for the ego vehicle (the UDP stream makes the angular component only available for the ego vehicle)
geometry_msgs/Twist ego_velocity
#Linear projection distance along the centerline of the track for the ego vehicle, negative implies behind starting line, positive implies forward past the starting line
float64 ego_track_progress