#include "deepracing_ros/utils/f1_msg_utils_2023.h"
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <map>
#include <memory>
#include <regex>
#include <string>
#include <utility>
#include <vector>
#include <exception>
#include <algorithm>

const std::string deepracing_ros::F1MsgUtils2023::world_coordinate_name("track");
const std::string deepracing_ros::F1MsgUtils2023::car_coordinate_name("centroid");
deepracing_msgs::msg::MarshalZone deepracing_ros::F1MsgUtils2023::toROS(const deepf1::twenty_twentythree::MarshalZone& marshal_zone)
{
  deepracing_msgs::msg::MarshalZone rtn;
  rtn.zone_flag = marshal_zone.zoneFlag;
  rtn.zone_start = marshal_zone.zoneStart;
  return rtn;
}


deepracing_msgs::msg::CarSetupData deepracing_ros::F1MsgUtils2023::toROS(const deepf1::twenty_twentythree::CarSetupData& setup_data)
{
  deepracing_msgs::msg::CarSetupData rtn;
  rtn.ballast = setup_data.ballast;
  rtn.brake_bias = setup_data.brakeBias;
  rtn.brake_pressure = setup_data.brakePressure;
  rtn.front_anti_roll_bar = setup_data.frontAntiRollBar;
  rtn.front_camber = setup_data.frontCamber;
  rtn.front_suspension = setup_data.frontSuspension;
  rtn.front_suspension_height = setup_data.frontSuspensionHeight;
  rtn.front_toe = setup_data.frontToe;
  rtn.front_wing = setup_data.frontWing;
  rtn.fuel_load = setup_data.fuelLoad;
  rtn.off_throttle = setup_data.offThrottle;
  rtn.on_throttle = setup_data.onThrottle;
  rtn.rear_anti_roll_bar = setup_data.rearAntiRollBar;
  rtn.rear_camber = setup_data.rearCamber;
  rtn.rear_suspension = setup_data.rearSuspension;
  rtn.rear_suspension_height = setup_data.rearSuspensionHeight;
  rtn.rear_toe = setup_data.rearToe;

  rtn.front_left_tyre_pressure = setup_data.frontLeftTyrePressure;
  rtn.front_right_tyre_pressure = setup_data.frontRightTyrePressure;
  rtn.rear_left_tyre_pressure = setup_data.rearLeftTyrePressure;
  rtn.rear_right_tyre_pressure = setup_data.rearRightTyrePressure;

  rtn.rear_wing = setup_data.rearWing;
  return rtn;
}
deepracing_msgs::msg::PacketCarSetupData deepracing_ros::F1MsgUtils2023::toROS(const deepf1::twenty_twentythree::PacketCarSetupData& packet_setup_data, bool copy_all_cars)
{
  deepracing_msgs::msg::PacketCarSetupData rtn;
  rtn.header = deepracing_ros::F1MsgUtils2023::toROS(packet_setup_data.header);
  if (rtn.header.player_car_index<22)
  {
    rtn.car_setup_data[rtn.header.player_car_index] = deepracing_ros::F1MsgUtils2023::toROS(packet_setup_data.carSetups[rtn.header.player_car_index]);
  }
  if (rtn.header.secondary_player_car_index<22)
  {
    rtn.car_setup_data[rtn.header.secondary_player_car_index] = deepracing_ros::F1MsgUtils2023::toROS(packet_setup_data.carSetups[rtn.header.secondary_player_car_index]);
  }
  if(copy_all_cars)
  {
    for(unsigned int i =0; i < 22; i++)
    {
      if (i!=rtn.header.player_car_index)
      {
        rtn.car_setup_data[i] = deepracing_ros::F1MsgUtils2023::toROS(packet_setup_data.carSetups[i]);
      }
    }
  }
  return rtn;
}


deepracing_msgs::msg::CarStatusData deepracing_ros::F1MsgUtils2023::toROS(const deepf1::twenty_twentythree::CarStatusData& status_data)
{
  deepracing_msgs::msg::CarStatusData rtn;
  rtn.anti_lock_brakes = status_data.antiLockBrakes;
  rtn.drs_allowed = status_data.drsAllowed;
  rtn.ers_deploy_mode = status_data.ersDeployMode;
  rtn.ers_deployed_this_lap = status_data.ersDeployedThisLap;
  rtn.ers_harvested_this_lap_mguh = status_data.ersHarvestedThisLapMGUH;
  rtn.ers_harvested_this_lap_mguk = status_data.ersHarvestedThisLapMGUK;
  rtn.ers_store_energy = status_data.ersStoreEnergy;
  rtn.front_brake_bias = status_data.frontBrakeBias;
  rtn.fuel_capacity = status_data.fuelCapacity;
  rtn.fuel_in_tank = status_data.fuelInTank;
  rtn.fuel_mix = status_data.fuelMix;
  rtn.idle_rpm = status_data.idleRPM;
  rtn.max_gears = status_data.maxGears;
  rtn.max_rpm = status_data.maxRPM;
  rtn.pit_limiter_status = status_data.pitLimiterStatus;
  rtn.traction_control = status_data.tractionControl;
  rtn.vehicle_fia_flags = status_data.vehicleFiaFlags;
  rtn.actual_tyre_compound = status_data.actualTyreCompound;
  rtn.visual_tyre_compound = status_data.visualTyreCompound;
  // std::copy_n(&(status_data.tyresDamage[0]), 4, rtn.tyres_damage.begin());
  // std::copy_n(&(status_data.tyresWear[0]), 4, rtn.tyres_wear.begin());
  return rtn;
}
deepracing_msgs::msg::PacketCarStatusData deepracing_ros::F1MsgUtils2023::toROS(const deepf1::twenty_twentythree::PacketCarStatusData& packet_status_data, bool copy_all_cars)
{

  deepracing_msgs::msg::PacketCarStatusData rtn;
  rtn.header = deepracing_ros::F1MsgUtils2023::toROS(packet_status_data.header);
  if (rtn.header.player_car_index<22)
  {
    rtn.car_status_data[rtn.header.player_car_index] = deepracing_ros::F1MsgUtils2023::toROS(packet_status_data.carStatusData[rtn.header.player_car_index]);
  }
  if (rtn.header.secondary_player_car_index<22)
  {
    rtn.car_status_data[rtn.header.secondary_player_car_index] = deepracing_ros::F1MsgUtils2023::toROS(packet_status_data.carStatusData[rtn.header.secondary_player_car_index]);
  }
  if(copy_all_cars)
  {
    for(unsigned int i =0; i < 22; i++)
    {
      if (i!=rtn.header.player_car_index)
      {
        rtn.car_status_data[i] = deepracing_ros::F1MsgUtils2023::toROS(packet_status_data.carStatusData[i]);
      }
    }
  }
  return rtn;
}


deepracing_msgs::msg::PacketLapData deepracing_ros::F1MsgUtils2023::toROS(const deepf1::twenty_twentythree::PacketLapData& lap_data, bool copy_all_cars)
{
  deepracing_msgs::msg::PacketLapData rtn;
  rtn.header = deepracing_ros::F1MsgUtils2023::toROS(lap_data.header);
  if (rtn.header.player_car_index<22)
  {
    rtn.lap_data[rtn.header.player_car_index] = deepracing_ros::F1MsgUtils2023::toROS(lap_data.lapData[rtn.header.player_car_index]);
  }
  if (rtn.header.secondary_player_car_index<22)
  {
    rtn.lap_data[rtn.header.secondary_player_car_index] = deepracing_ros::F1MsgUtils2023::toROS(lap_data.lapData[rtn.header.secondary_player_car_index]);
  }
  if(copy_all_cars)
  {
    for(unsigned int i =0; i < 22; i++)
    {
      if (i!=rtn.header.player_car_index)
      {
        rtn.lap_data[i] = deepracing_ros::F1MsgUtils2023::toROS(lap_data.lapData[i]);
      }
    }
  }
  return rtn;
}
deepracing_msgs::msg::LapData deepracing_ros::F1MsgUtils2023::toROS(const deepf1::twenty_twentythree::LapData& lap_data)
{
  deepracing_msgs::msg::LapData rtn;
  rtn.corner_cutting_warnings = lap_data.cornerCuttingWarnings;
  rtn.car_position = lap_data.carPosition;
  rtn.current_lap_invalid = lap_data.currentLapInvalid;
  rtn.current_lap_num = lap_data.currentLapNum;
  rtn.current_lap_time_in_ms = lap_data.currentLapTimeInMS;
  rtn.driver_status = lap_data.driverStatus;
  rtn.grid_position = lap_data.gridPosition;
  rtn.lap_distance = lap_data.lapDistance;
  rtn.last_lap_time_in_ms = lap_data.lastLapTimeInMS;
  rtn.penalties = lap_data.penalties;
  rtn.pit_status = lap_data.pitStatus;
  rtn.result_status = lap_data.resultStatus;
  rtn.safety_car_delta = lap_data.safetyCarDelta;

  rtn.sector1_time_in_ms = lap_data.sector1TimeInMS;
  rtn.sector1_time_minutes = lap_data.sector1TimeMinutes;
  rtn.sector2_time_in_ms = lap_data.sector2TimeInMS;
  rtn.sector2_time_minutes = lap_data.sector2TimeMinutes;

  rtn.sector = lap_data.sector;
  rtn.total_distance = lap_data.totalDistance;
  return rtn;
}
deepracing_msgs::msg::WeatherForecastSample deepracing_ros::F1MsgUtils2023::toROS(const deepf1::twenty_twentythree::WeatherForecastSample& weather_forecast)
{
  deepracing_msgs::msg::WeatherForecastSample rtn;
  rtn.air_temperature = weather_forecast.airTemperature;
  rtn.session_type = weather_forecast.sessionType;
  rtn.time_offset = weather_forecast.timeOffset;
  rtn.track_temperature = weather_forecast.trackTemperature;
  rtn.weather = weather_forecast.weather;
  return rtn;
}
deepracing_msgs::msg::PacketSessionData deepracing_ros::F1MsgUtils2023::toROS(const deepf1::twenty_twentythree::PacketSessionData& session_data)
{
  deepracing_msgs::msg::PacketSessionData rtn;
  rtn.header=toROS(session_data.header);
  rtn.num_marshal_zones = session_data.numMarshalZones;
  for (uint8_t i = 0; i < rtn.num_marshal_zones; i++)
  {
    rtn.marshal_zones[i] = toROS(session_data.marshalZones[i]);
  }
  rtn.num_weather_forecast_samples = session_data.numWeatherForecastSamples;
  for (uint8_t i = 0; i < rtn.num_weather_forecast_samples; i++)
  {
    rtn.weather_forecast_samples[i] = toROS(session_data.weatherForecastSamples[i]);
  }
  rtn.air_temperature = session_data.airTemperature;
  rtn.era = session_data.formula;
  rtn.game_paused  = session_data.gamePaused;
  rtn.is_spectating = session_data.isSpectating;
  rtn.network_game = session_data.networkGame;
  rtn.pit_speed_limit = session_data.pitSpeedLimit;
  rtn.safety_car_status = session_data.safetyCarStatus;
  rtn.session_duration = session_data.sessionDuration;
  rtn.session_type = session_data.sessionType;
  rtn.session_time_left = session_data.sessionTimeLeft;
  rtn.spectator_car_index = session_data.spectatorCarIndex;
  rtn.track_id = session_data.trackId;
  rtn.track_length = session_data.trackLength;
  rtn.track_temperature = session_data.trackTemperature;
  return rtn;
}
deepracing_msgs::msg::CarTelemetryData deepracing_ros::F1MsgUtils2023::toROS(const deepf1::twenty_twentythree::CarTelemetryData& telemetry_data)
{
  deepracing_msgs::msg::CarTelemetryData  rtn;
  
  rtn.brake = telemetry_data.brake;
  rtn.clutch = telemetry_data.clutch;
  rtn.drs = telemetry_data.drs;
  rtn.engine_rpm = telemetry_data.engineRPM;
  rtn.engine_temperature = telemetry_data.engineTemperature;
  rtn.gear = telemetry_data.gear;
  rtn.rev_lights_percent = telemetry_data.revLightsPercent;
  rtn.speed = telemetry_data.speed;
  //We use a left-positive convention for steering angle, but the F1 UDP uses a right-positive.
  rtn.steer = -telemetry_data.steer;
  rtn.throttle = telemetry_data.throttle;
  std::copy_n(&(telemetry_data.brakesTemperature[0]), 4, rtn.brakes_temperature.begin());
  std::copy_n(&(telemetry_data.tyresInnerTemperature[0]), 4, rtn.tyres_inner_temperature.begin());
  std::copy_n(&(telemetry_data.tyresPressure[0]), 4, rtn.tyres_pressure.begin());
  std::copy_n(&(telemetry_data.tyresSurfaceTemperature[0]), 4, rtn.tyres_surface_temperature.begin());
  return rtn;
}
deepracing_msgs::msg::PacketCarTelemetryData deepracing_ros::F1MsgUtils2023::toROS(const deepf1::twenty_twentythree::PacketCarTelemetryData& telemetry_data, bool copy_all_cars)
{
  deepracing_msgs::msg::PacketCarTelemetryData rtn;
  rtn.header = deepracing_ros::F1MsgUtils2023::toROS(telemetry_data.header);
  if (rtn.header.player_car_index<22)
  {
    rtn.car_telemetry_data[rtn.header.player_car_index] = deepracing_ros::F1MsgUtils2023::toROS(telemetry_data.carTelemetryData[rtn.header.player_car_index]);
  }
  if (rtn.header.secondary_player_car_index<22)
  {
    rtn.car_telemetry_data[rtn.header.secondary_player_car_index] = deepracing_ros::F1MsgUtils2023::toROS(telemetry_data.carTelemetryData[rtn.header.secondary_player_car_index]);
  }
  if(copy_all_cars)
  {
    for(unsigned int i =0; i < 22; i++)
    {
      if (i!=rtn.header.player_car_index)
      {
        rtn.car_telemetry_data[i] = deepracing_ros::F1MsgUtils2023::toROS(telemetry_data.carTelemetryData[i]);
      }
    }
  }
  return rtn;
}
deepracing_msgs::msg::PacketHeader deepracing_ros::F1MsgUtils2023::toROS(const deepf1::twenty_twentythree::PacketHeader& header_data)
{
    deepracing_msgs::msg::PacketHeader rtn;
    
    rtn.frame_identifier = header_data.frameIdentifier;
    rtn.game_year = header_data.gameYear;
    rtn.game_major_version = header_data.gameMajorVersion;
    rtn.game_minor_version = header_data.gameMinorVersion;
    rtn.overall_frame_identifier = header_data.overallFrameIdentifier;
    rtn.packet_format = header_data.packetFormat;
    rtn.packet_id = header_data.packetId;
    rtn.packet_version = header_data.packetVersion;
    rtn.player_car_index = header_data.playerCarIndex;
    rtn.secondary_player_car_index = header_data.secondaryPlayerCarIndex;
    rtn.session_time = header_data.sessionTime;
    rtn.session_uid = header_data.sessionUID;

    return rtn;
}
deepracing_msgs::msg::PacketMotionExData toROS(const deepf1::twenty_twentythree::PacketMotionExData& motion_ex_data)
{
  
    deepracing_msgs::msg::PacketMotionExData rtn;
    rtn.header = deepracing_ros::F1MsgUtils2023::toROS(motion_ex_data.header);
    rtn.angular_acceleration.x = motion_ex_data.angularAccelerationX;
    rtn.angular_acceleration.y = motion_ex_data.angularAccelerationY;
    rtn.angular_acceleration.z = motion_ex_data.angularAccelerationZ;
    rtn.angular_velocity.x = motion_ex_data.angularVelocityX;
    rtn.angular_velocity.y = motion_ex_data.angularVelocityY;
    rtn.angular_velocity.z = motion_ex_data.angularVelocityZ;
    rtn.local_velocity.x = motion_ex_data.localVelocityX;
    rtn.local_velocity.y = motion_ex_data.localVelocityY;
    rtn.local_velocity.z = motion_ex_data.localVelocityZ;
    rtn.front_wheels_angle = motion_ex_data.frontWheelsAngle;
    std::copy_n(std::cbegin(motion_ex_data.wheelSlipAngle), 4, rtn.wheel_slip_angle.begin());
    std::copy_n(std::cbegin(motion_ex_data.wheelSlipRatio), 4, rtn.wheel_slip_ratio.begin());
    std::copy_n(std::cbegin(motion_ex_data.wheelSpeed), 4, rtn.wheel_speed.begin());
    std::copy_n(std::cbegin(motion_ex_data.suspensionAcceleration), 4, rtn.suspension_acceleration.begin());
    std::copy_n(std::cbegin(motion_ex_data.suspensionVelocity), 4, rtn.suspension_velocity.begin());
    std::copy_n(std::cbegin(motion_ex_data.suspensionPosition), 4, rtn.suspension_position.begin());
    std::copy_n(std::cbegin(motion_ex_data.wheelLatForce), 4, rtn.wheel_lat_force.begin());
    std::copy_n(std::cbegin(motion_ex_data.wheelLongForce), 4, rtn.wheel_long_force.begin());
    std::copy_n(std::cbegin(motion_ex_data.wheelVertForce), 4, rtn.wheel_vert_force.begin());
    return rtn;
}
deepracing_msgs::msg::PacketMotionData deepracing_ros::F1MsgUtils2023::toROS(const deepf1::twenty_twentythree::PacketMotionData& motion_data, bool copy_all_cars)
{
    deepracing_msgs::msg::PacketMotionData rtn;
    rtn.header = deepracing_ros::F1MsgUtils2023::toROS(motion_data.header);    
    if(copy_all_cars)
    {
      auto beg = std::cbegin<deepf1::twenty_twentythree::CarMotionData [22]>(motion_data.carMotionData);
      auto end = std::cend<deepf1::twenty_twentythree::CarMotionData [22]>(motion_data.carMotionData);
      std::function<deepracing_msgs::msg::CarMotionData (const deepf1::twenty_twentythree::CarMotionData&)> f = &deepracing_ros::F1MsgUtils2023::toROSMotionData;
      std::transform(beg, end, rtn.car_motion_data.begin(), f);
      return rtn;
    }
    if(rtn.header.player_car_index<22)
    {
      rtn.car_motion_data[rtn.header.player_car_index] = deepracing_ros::F1MsgUtils2023::toROSMotionData(motion_data.carMotionData[rtn.header.player_car_index]);
    }
    if (rtn.header.secondary_player_car_index<22)
    {
      rtn.car_motion_data[rtn.header.secondary_player_car_index] = deepracing_ros::F1MsgUtils2023::toROSMotionData(motion_data.carMotionData[rtn.header.secondary_player_car_index]);
    }
    return rtn;
}
deepracing_msgs::msg::CarMotionData deepracing_ros::F1MsgUtils2023::toROSMotionData(const deepf1::twenty_twentythree::CarMotionData& motion_data)
{
    deepracing_msgs::msg::CarMotionData rtn;
    Eigen::Vector3d forwardVec( (double)motion_data.worldForwardDirX, (double)motion_data.worldForwardDirY, (double)motion_data.worldForwardDirZ );
    forwardVec.normalize();
    rtn.world_forward_dir.header.frame_id=world_coordinate_name;
 //   rtn.world_forward_dir.header.stamp = rostime;
    rtn.world_forward_dir.vector.x = forwardVec.x();
    rtn.world_forward_dir.vector.y = forwardVec.y();
    rtn.world_forward_dir.vector.z = forwardVec.z();
    Eigen::Vector3d rightVec( (double)motion_data.worldRightDirX, (double)motion_data.worldRightDirY, (double)motion_data.worldRightDirZ );
    rightVec.normalize();
    Eigen::Vector3d leftVec( -rightVec );
    Eigen::Vector3d upVec = forwardVec.cross(leftVec);
    upVec.normalize();
    rtn.world_up_dir.header.frame_id=world_coordinate_name;
  //  rtn.world_up_dir.header.stamp = rostime;
    rtn.world_up_dir.vector.x = upVec.x();
    rtn.world_up_dir.vector.y = upVec.y();
    rtn.world_up_dir.vector.z = upVec.z();
    rtn.world_right_dir.header.frame_id=world_coordinate_name;
    //rtn.world_right_dir.header.stamp = rostime;
    rtn.world_right_dir.vector.x = rightVec.x();
    rtn.world_right_dir.vector.y = rightVec.y();
    rtn.world_right_dir.vector.z = rightVec.z();


    rtn.world_position.header.frame_id=world_coordinate_name;
 //   rtn.world_position.header.stamp = rostime;
    rtn.world_position.point.x = motion_data.worldPositionX;
    rtn.world_position.point.y = motion_data.worldPositionY;
    rtn.world_position.point.z = motion_data.worldPositionZ;


    rtn.world_velocity.header.frame_id=world_coordinate_name;
  //  rtn.world_velocity.header.stamp = rostime;
    rtn.world_velocity.vector.x = motion_data.worldVelocityX;
    rtn.world_velocity.vector.y = motion_data.worldVelocityY;
    rtn.world_velocity.vector.z = motion_data.worldVelocityZ;

    rtn.g_force_lateral = motion_data.gForceLateral;
    rtn.g_force_longitudinal = motion_data.gForceLongitudinal;
    rtn.g_force_vertical = motion_data.gForceVertical;

    rtn.roll = motion_data.roll;
    rtn.pitch = motion_data.pitch;
    rtn.yaw = motion_data.yaw;
    return rtn;
}