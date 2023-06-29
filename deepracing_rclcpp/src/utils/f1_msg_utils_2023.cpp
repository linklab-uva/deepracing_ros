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
  if (rtn.header.player_car_index<rtn.car_setup_data.size())
  {
    rtn.car_setup_data[rtn.header.player_car_index] = deepracing_ros::F1MsgUtils2023::toROS(packet_setup_data.carSetups[rtn.header.player_car_index]);
  }
  if (rtn.header.secondary_player_car_index<rtn.car_setup_data.size())
  {
    rtn.car_setup_data[rtn.header.secondary_player_car_index] = deepracing_ros::F1MsgUtils2023::toROS(packet_setup_data.carSetups[rtn.header.secondary_player_car_index]);
  }
  if(copy_all_cars)
  {
    for(unsigned int i =0; i < rtn.car_setup_data.size(); i++)
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
  rtn.actual_tyre_compound = status_data.actualTyreCompound;
  rtn.anti_lock_brakes = status_data.antiLockBrakes;
  rtn.drs_activation_distance = status_data.drsActivationDistance;
  rtn.drs_allowed = status_data.drsAllowed;
  rtn.engine_power_ice = status_data.enginePowerICE;
  rtn.engine_power_mguk = status_data.enginePowerMGUK;
  rtn.ers_deploy_mode = status_data.ersDeployMode;
  rtn.ers_deployed_this_lap = status_data.ersDeployedThisLap;
  rtn.ers_harvested_this_lap_mguh = status_data.ersHarvestedThisLapMGUH;
  rtn.ers_harvested_this_lap_mguk = status_data.ersHarvestedThisLapMGUK;
  rtn.ers_store_energy = status_data.ersStoreEnergy;
  rtn.front_brake_bias = status_data.frontBrakeBias;
  rtn.fuel_capacity = status_data.fuelCapacity;
  rtn.fuel_in_tank = status_data.fuelInTank;
  rtn.fuel_mix = status_data.fuelMix;
  rtn.fuel_remaining_laps = status_data.fuelRemainingLaps;
  rtn.idle_rpm = status_data.idleRPM;
  rtn.max_gears = status_data.maxGears;
  rtn.max_rpm = status_data.maxRPM;
  rtn.network_paused = status_data.networkPaused;
  rtn.pit_limiter_status = status_data.pitLimiterStatus;
  rtn.traction_control = status_data.tractionControl;
  rtn.tyres_age_laps = status_data.tyresAgeLaps;
  rtn.vehicle_fia_flags = status_data.vehicleFiaFlags;
  rtn.visual_tyre_compound = status_data.visualTyreCompound;
  return rtn;
}
deepracing_msgs::msg::PacketCarStatusData deepracing_ros::F1MsgUtils2023::toROS(const deepf1::twenty_twentythree::PacketCarStatusData& packet_status_data, bool copy_all_cars)
{

  deepracing_msgs::msg::PacketCarStatusData rtn;
  rtn.header = deepracing_ros::F1MsgUtils2023::toROS(packet_status_data.header);
  if (rtn.header.player_car_index<rtn.car_status_data.size())
  {
    rtn.car_status_data[rtn.header.player_car_index] = deepracing_ros::F1MsgUtils2023::toROS(packet_status_data.carStatusData[rtn.header.player_car_index]);
  }
  if (rtn.header.secondary_player_car_index<rtn.car_status_data.size())
  {
    rtn.car_status_data[rtn.header.secondary_player_car_index] = deepracing_ros::F1MsgUtils2023::toROS(packet_status_data.carStatusData[rtn.header.secondary_player_car_index]);
  }
  if(copy_all_cars)
  {
    for(unsigned int i =0; i < rtn.car_status_data.size(); i++)
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
  if (rtn.header.player_car_index<rtn.lap_data.size())
  {
    rtn.lap_data[rtn.header.player_car_index] = deepracing_ros::F1MsgUtils2023::toROS(lap_data.lapData[rtn.header.player_car_index]);
  }
  if (rtn.header.secondary_player_car_index<rtn.lap_data.size())
  {
    rtn.lap_data[rtn.header.secondary_player_car_index] = deepracing_ros::F1MsgUtils2023::toROS(lap_data.lapData[rtn.header.secondary_player_car_index]);
  }
  if(copy_all_cars)
  {
    for(unsigned int i =0; i < rtn.lap_data.size(); i++)
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
  rtn.result_status = lap_data.resultStatus;
  if((rtn.result_status==0) || (rtn.result_status==1))
  {
    //This car is not producing valid data, the rest of the fields are garbage.
    return rtn;
  }
  rtn.car_position = lap_data.carPosition;
  rtn.corner_cutting_warnings = lap_data.cornerCuttingWarnings;
  rtn.current_lap_invalid = lap_data.currentLapInvalid;
  rtn.current_lap_num = lap_data.currentLapNum;
  rtn.current_lap_time_in_ms = lap_data.currentLapTimeInMS;
  rtn.delta_to_car_in_front_in_ms = lap_data.deltaToCarInFrontInMS;
  rtn.delta_to_race_leader_in_ms = lap_data.deltaToRaceLeaderInMS;
  rtn.driver_status = lap_data.driverStatus;
  rtn.grid_position = lap_data.gridPosition;
  rtn.lap_distance = lap_data.lapDistance;
  rtn.last_lap_time_in_ms = lap_data.lastLapTimeInMS;
  rtn.num_pit_stops = lap_data.numPitStops;
  rtn.num_unserved_drive_through_pens = lap_data.numUnservedDriveThroughPens;
  rtn.num_unserved_stop_go_pens = lap_data.numUnservedStopGoPens;
  rtn.penalties = lap_data.penalties;
  rtn.pit_lane_time_in_lane_in_ms = lap_data.pitLaneTimeInLaneInMS;
  rtn.pit_lane_timer_active = lap_data.pitLaneTimerActive;
  rtn.pit_status = lap_data.pitStatus;
  rtn.safety_car_delta = lap_data.safetyCarDelta;
  rtn.sector1_time_in_ms = lap_data.sector1TimeInMS;
  rtn.sector1_time_minutes = lap_data.sector1TimeMinutes;
  rtn.sector2_time_in_ms = lap_data.sector2TimeInMS;
  rtn.sector2_time_minutes = lap_data.sector2TimeMinutes;
  rtn.sector = lap_data.sector;
  rtn.total_distance = lap_data.totalDistance;
  rtn.total_warnings = lap_data.totalWarnings;
  return rtn;
}
deepracing_msgs::msg::WeatherForecastSample deepracing_ros::F1MsgUtils2023::toROS(const deepf1::twenty_twentythree::WeatherForecastSample& weather_forecast)
{
  deepracing_msgs::msg::WeatherForecastSample rtn;
  rtn.air_temperature = weather_forecast.airTemperature;
  rtn.air_temperature_change = weather_forecast.airTemperatureChange;
  rtn.rain_percentage = weather_forecast.rainPercentage;
  rtn.session_type = weather_forecast.sessionType;
  rtn.time_offset = weather_forecast.timeOffset;
  rtn.track_temperature = weather_forecast.trackTemperature;
  rtn.track_temperature_change = weather_forecast.trackTemperatureChange;
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

  rtn.ai_difficulty = session_data.aiDifficulty;
  rtn.braking_assist = session_data.brakingAssist;
  rtn.drs_assist = session_data.DRSAssist;
  rtn.dynamic_racing_line = session_data.dynamicRacingLine;
  rtn.dynamic_racing_line_type = session_data.dynamicRacingLineType;    
  rtn.ers_assist = session_data.ERSAssist;
  rtn.forecast_accuracy = session_data.forecastAccuracy;
  rtn.game_mode = session_data.gameMode;
  rtn.gearbox_assist = session_data.gearboxAssist;
  rtn.num_red_flag_periods = session_data.numRedFlagPeriods;
  rtn.num_safety_car_periods = session_data.numSafetyCarPeriods;
  rtn.num_virtual_safety_car_periods = session_data.numVirtualSafetyCarPeriods;
  rtn.pit_assist = session_data.pitAssist;
  rtn.pit_release_assist = session_data.pitReleaseAssist;
  rtn.pit_speed_limit = session_data.pitSpeedLimit;
  rtn.pit_stop_rejoin_position = session_data.pitStopRejoinPosition;
  rtn.pit_stop_window_ideal_lap = session_data.pitStopWindowIdealLap;
  rtn.pit_stop_window_latest_lap = session_data.pitStopWindowLatestLap;
  rtn.rule_set = session_data.ruleSet;
  rtn.session_duration = session_data.sessionDuration;
  rtn.session_length = session_data.sessionLength;
  rtn.sli_pro_native_support = session_data.sliProNativeSupport;
  rtn.steering_assist = session_data.steeringAssist;
  rtn.temperature_units_lead_player = session_data.temperatureUnitsLeadPlayer;
  rtn.temperature_units_secondary_player = session_data.temperatureUnitsSecondaryPlayer;
  rtn.time_of_day = session_data.timeOfDay;
  rtn.total_laps = session_data.totalLaps;
  rtn.weekend_link_identifier = session_data.weekendLinkIdentifier; 
  
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
  rtn.rev_lights_bit_value = telemetry_data.revLightsBitValue;
  rtn.rev_lights_percent = telemetry_data.revLightsPercent;
  rtn.speed = telemetry_data.speed;
  //We use a left-positive convention for steering angle, but the F1 UDP uses a right-positive.
  rtn.steer = -telemetry_data.steer;
  rtn.throttle = telemetry_data.throttle;
  for(std::size_t i = 0; i < 4; i++)
  {
    rtn.brakes_temperature[i] = telemetry_data.brakesTemperature[i];
    rtn.tyres_inner_temperature[i] = telemetry_data.tyresInnerTemperature[i];
    rtn.tyres_pressure[i] = telemetry_data.tyresPressure[i];
    rtn.tyres_surface_temperature[i] = telemetry_data.tyresSurfaceTemperature[i];
    rtn.surface_type[i] = telemetry_data.surfaceType[i];
  }
  return rtn;
}
deepracing_msgs::msg::PacketCarTelemetryData deepracing_ros::F1MsgUtils2023::toROS(const deepf1::twenty_twentythree::PacketCarTelemetryData& telemetry_data, bool copy_all_cars)
{
  deepracing_msgs::msg::PacketCarTelemetryData rtn;
  rtn.header = deepracing_ros::F1MsgUtils2023::toROS(telemetry_data.header);
  if (rtn.header.player_car_index<rtn.car_telemetry_data.size())
  {
    rtn.car_telemetry_data[rtn.header.player_car_index] = deepracing_ros::F1MsgUtils2023::toROS(telemetry_data.carTelemetryData[rtn.header.player_car_index]);
  }
  if (rtn.header.secondary_player_car_index<rtn.car_telemetry_data.size())
  {
    rtn.car_telemetry_data[rtn.header.secondary_player_car_index] = deepracing_ros::F1MsgUtils2023::toROS(telemetry_data.carTelemetryData[rtn.header.secondary_player_car_index]);
  }
  if(copy_all_cars)
  {
    for(unsigned int i =0; i < rtn.car_telemetry_data.size(); i++)
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
deepracing_msgs::msg::PacketMotionExData deepracing_ros::F1MsgUtils2023::toROS(const deepf1::twenty_twentythree::PacketMotionExData& motion_ex_data)
{
    deepracing_msgs::msg::PacketMotionExData rtn;
    rtn.header = deepracing_ros::F1MsgUtils2023::toROS(motion_ex_data.header);
    rtn.angular_acceleration.x = motion_ex_data.angularAccelerationX;
    rtn.angular_acceleration.y = motion_ex_data.angularAccelerationY;
    rtn.angular_acceleration.z = motion_ex_data.angularAccelerationZ;
    rtn.angular_velocity.x = motion_ex_data.angularVelocityX;
    rtn.angular_velocity.y = motion_ex_data.angularVelocityY;
    rtn.angular_velocity.z = motion_ex_data.angularVelocityZ;
    rtn.local_velocity.x = motion_ex_data.localVelocityZ;
    rtn.local_velocity.y = motion_ex_data.localVelocityX;
    rtn.local_velocity.z = motion_ex_data.localVelocityY;
    rtn.front_wheels_angle = motion_ex_data.frontWheelsAngle;
    rtn.height_of_cg_above_ground = motion_ex_data.heightOfCOGAboveGround;
    for(std::size_t i = 0; i < 4; i++)
    {
      rtn.wheel_slip_angle[i] = motion_ex_data.wheelSlipAngle[i];
      rtn.wheel_slip_ratio[i] = motion_ex_data.wheelSlipRatio[i];
      rtn.wheel_speed[i] = motion_ex_data.wheelSpeed[i];
      rtn.suspension_acceleration[i] = motion_ex_data.suspensionAcceleration[i];
      rtn.suspension_velocity[i] = motion_ex_data.suspensionVelocity[i];
      rtn.suspension_position[i] = motion_ex_data.suspensionPosition[i];
      rtn.wheel_lat_force[i] = motion_ex_data.wheelLatForce[i];
      rtn.wheel_long_force[i] = motion_ex_data.wheelLongForce[i];
      rtn.wheel_vert_force[i] = motion_ex_data.wheelVertForce[i];
    }
    return rtn;
}
deepracing_msgs::msg::PacketMotionData deepracing_ros::F1MsgUtils2023::toROS(const deepf1::twenty_twentythree::PacketMotionData& motion_data, bool copy_all_cars)
{
    deepracing_msgs::msg::PacketMotionData rtn;
    rtn.header = deepracing_ros::F1MsgUtils2023::toROS(motion_data.header);    
    if(copy_all_cars)
    {
      auto beg = std::cbegin<deepf1::twenty_twentythree::CarMotionData [rtn.car_motion_data.size()]>(motion_data.carMotionData);
      auto end = std::cend<deepf1::twenty_twentythree::CarMotionData [rtn.car_motion_data.size()]>(motion_data.carMotionData);
      std::function<deepracing_msgs::msg::CarMotionData (const deepf1::twenty_twentythree::CarMotionData&)> f = &deepracing_ros::F1MsgUtils2023::toROSMotionData;
      std::transform(beg, end, rtn.car_motion_data.begin(), f);
      return rtn;
    }
    if(rtn.header.player_car_index<rtn.car_motion_data.size())
    {
      rtn.car_motion_data[rtn.header.player_car_index] = deepracing_ros::F1MsgUtils2023::toROSMotionData(motion_data.carMotionData[rtn.header.player_car_index]);
    }
    if (rtn.header.secondary_player_car_index<rtn.car_motion_data.size())
    {
      rtn.car_motion_data[rtn.header.secondary_player_car_index] = deepracing_ros::F1MsgUtils2023::toROSMotionData(motion_data.carMotionData[rtn.header.secondary_player_car_index]);
    }
    return rtn;
}
deepracing_msgs::msg::CarMotionData deepracing_ros::F1MsgUtils2023::toROSMotionData(const deepf1::twenty_twentythree::CarMotionData& motion_data)
{
    deepracing_msgs::msg::CarMotionData rtn;
    Eigen::Vector3d forwardVec = 
      Eigen::Vector3d( (double)motion_data.worldForwardDirX, (double)motion_data.worldForwardDirY, (double)motion_data.worldForwardDirZ ).normalized();
    rtn.world_forward_dir.header.frame_id=world_coordinate_name;
    rtn.world_forward_dir.vector.x = forwardVec.x();
    rtn.world_forward_dir.vector.y = forwardVec.y();
    rtn.world_forward_dir.vector.z = forwardVec.z();
    Eigen::Vector3d leftVec = 
      (-Eigen::Vector3d( (double)motion_data.worldRightDirX, (double)motion_data.worldRightDirY, (double)motion_data.worldRightDirZ )).normalized();
    Eigen::Vector3d upVec = forwardVec.cross(leftVec).normalized();
    leftVec = upVec.cross(forwardVec).normalized();
    rtn.world_up_dir.header.frame_id=world_coordinate_name;
    rtn.world_up_dir.vector.x = upVec.x();
    rtn.world_up_dir.vector.y = upVec.y();
    rtn.world_up_dir.vector.z = upVec.z();
    rtn.world_left_dir.header.frame_id=world_coordinate_name;
    rtn.world_left_dir.vector.x = leftVec.x();
    rtn.world_left_dir.vector.y = leftVec.y();
    rtn.world_left_dir.vector.z = leftVec.z();


    rtn.world_position.header.frame_id=world_coordinate_name;
    rtn.world_position.point.x = motion_data.worldPositionX;
    rtn.world_position.point.y = motion_data.worldPositionY;
    rtn.world_position.point.z = motion_data.worldPositionZ;


    rtn.world_velocity.header.frame_id=world_coordinate_name;
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