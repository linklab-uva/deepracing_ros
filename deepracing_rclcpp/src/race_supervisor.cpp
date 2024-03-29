#include <rclcpp/rclcpp.hpp>
#include <random_numbers/random_numbers.h>
#include <deepracing_msgs/msg/timestamped_packet_car_telemetry_data.hpp>
#include <deepracing_msgs/msg/timestamped_packet_car_status_data.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <deepracing_msgs/msg/race_supervisor_state.hpp>
#include <deepracing_msgs/msg/driver_states.hpp>
#include <deepracing_msgs/srv/set_raceline.hpp>
#include <deepracing_msgs/srv/get_raceline.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class RaceSupervisorNode : public rclcpp::Node 
{

  public:
    RaceSupervisorNode( const rclcpp::NodeOptions & options )
     : rclcpp::Node("race_supervisor", options), m_time_of_next_overtake_(0)
    {
      m_frequency_ = declare_parameter<double>("frequency", 10.0);
      m_current_state_.description = "Uninitialized";
      m_current_state_.current_state = deepracing_msgs::msg::RaceSupervisorState::STATE_UNINITIALIZED;
      m_current_state_publisher_ = create_publisher<deepracing_msgs::msg::RaceSupervisorState>("race_supervisor_state", 1);
      
      rcl_interfaces::msg::ParameterDescriptor car_names_descriptor;
      car_names_descriptor.set__name("car_names");
      car_names_descriptor.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY);
      car_names_descriptor.set__read_only(false);
      car_names_descriptor.set__description("The names of the two cars to manage, each should have it's own namespace of various ROS2 schtuff");
      std::vector<std::string> car_names = declare_parameter<std::vector<std::string>>(car_names_descriptor.name, {"player1", "player2"}, car_names_descriptor);
      m_car1_name_ = car_names[0];

      std::string driver_states_topic = "/driver_states";
      m_driver_states_listener_ = this->create_subscription<deepracing_msgs::msg::DriverStates>(driver_states_topic, 1, std::bind(&RaceSupervisorNode::states_CB_, this, std::placeholders::_1));
     


      std::string car1_get_raceline_service = "/" + m_car1_name_ + "/get_raceline";
      m_car1_raceline_getter_ = create_client<deepracing_msgs::srv::GetRaceline>(car1_get_raceline_service);
      std::string car1_set_raceline_service = "/" + m_car1_name_ + "/set_raceline";
      m_car1_raceline_setter_ = create_client<deepracing_msgs::srv::SetRaceline>(car1_set_raceline_service);
      std::string car1_set_params_service = "/" + m_car1_name_ + "/control_node/set_parameters";
      m_car1_param_setter_ = create_client<rcl_interfaces::srv::SetParameters>(car1_set_params_service);

      m_car2_name_ = car_names[1];
       std::string car2_get_raceline_service = "/" + m_car2_name_ + "/get_raceline";
      m_car2_raceline_getter_ = create_client<deepracing_msgs::srv::GetRaceline>(car2_get_raceline_service);
      std::string car2_set_raceline_service = "/" + m_car2_name_ + "/set_raceline";
      m_car2_raceline_setter_ = create_client<deepracing_msgs::srv::SetRaceline>(car2_set_raceline_service);
      std::string car2_set_params_service = "/" + m_car2_name_ + "/control_node/set_parameters";
      m_car2_param_setter_ = create_client<rcl_interfaces::srv::SetParameters>(car2_set_params_service);

      rcl_interfaces::msg::ParameterDescriptor player_indices_descriptor;
      player_indices_descriptor.set__name("player_indices");
      player_indices_descriptor.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY);
      player_indices_descriptor.set__read_only(true);
      std::vector<int64_t> player_indices = declare_parameter< std::vector<int64_t> >(player_indices_descriptor.name, {-1, -2}, player_indices_descriptor);
      m_car1_index_ = player_indices.at(0);
      m_car2_index_ = player_indices.at(1);

    }
    void start()
    {
      m_timer_ = create_wall_timer(std::chrono::microseconds((uint64_t)std::round(1.0E6/m_frequency_)), std::bind(&RaceSupervisorNode::main_loop, this));
    }
  private:
    void main_loop()
    {
      RCLCPP_DEBUG(this->get_logger(), "Race supervisor main loop");
      if (m_current_state_.current_state==deepracing_msgs::msg::RaceSupervisorState::STATE_UNINITIALIZED)
      {
        handle_uninitialized();
      }
      else if (m_current_state_.current_state==deepracing_msgs::msg::RaceSupervisorState::STATE_FOLLOWING_RACELINES)
      {
        handle_following_racelines();
      }
      else if (m_current_state_.current_state==deepracing_msgs::msg::RaceSupervisorState::STATE_OVERTAKE)
      {
        handle_overtaking();
      }
      m_current_state_publisher_->publish(m_current_state_);
    }
    void set_car1_speedfactor(double factor)
    {
      rcl_interfaces::srv::SetParameters::Request::SharedPtr paramsreq(new rcl_interfaces::srv::SetParameters::Request);
      rcl_interfaces::msg::Parameter fullspeed;
      rcl_interfaces::msg::ParameterValue fullspeedval;
      fullspeedval.set__double_value(factor);
      fullspeedval.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);
      fullspeed.set__name("scale_factor");
      fullspeed.set__value(fullspeedval);
      paramsreq->parameters.push_back(fullspeed);
      RCLCPP_DEBUG(this->get_logger(), "Calling set params service");
      m_car1_param_setter_->async_send_request(paramsreq, std::bind(&RaceSupervisorNode::handle_car1_serviceresponse, this, std::placeholders::_1));
    }
    void handle_car1_serviceresponse(rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future)
    {
      RCLCPP_DEBUG(this->get_logger(), "Got result. Successful: %u Reason: %s", (uint8_t)future.get()->results.at(0).successful, future.get()->results.at(0).reason.c_str());
    }
    void set_car2_speedfactor(double factor)
    {
      rcl_interfaces::srv::SetParameters::Request::SharedPtr paramsreq(new rcl_interfaces::srv::SetParameters::Request);
      rcl_interfaces::msg::Parameter fullspeed;
      rcl_interfaces::msg::ParameterValue fullspeedval;
      fullspeedval.set__double_value(factor);
      fullspeedval.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);
      fullspeed.set__name("scale_factor");
      fullspeed.set__value(fullspeedval);
      paramsreq->parameters.push_back(fullspeed);
      RCLCPP_DEBUG(this->get_logger(), "Calling set params service");
      m_car2_param_setter_->async_send_request(paramsreq, std::bind(&RaceSupervisorNode::handle_car2_serviceresponse, this, std::placeholders::_1));
    }
    void handle_car2_serviceresponse(rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future)
    {
      RCLCPP_DEBUG(this->get_logger(), "Got result. Successful: %u Reason: %s", (uint8_t)future.get()->results.at(0).successful, future.get()->results.at(0).reason.c_str());
    }
    void handle_following_racelines()
    {
      if(!m_driver_states_)
      {

        RCLCPP_ERROR(this->get_logger(), "Haven't received any driver states data");
        return;
      }
      if (m_driver_states_->ego_lap_number<2  ||  m_driver_states_->secondary_vehicle_lap_number<2)
      {
        RCLCPP_DEBUG(this->get_logger(), "don't start overtaking until lap 2");
        return;
      }
      if( this->get_clock()->now() > m_time_of_next_overtake_)
      {
        RCLCPP_DEBUG(this->get_logger(), "Starting an overtake.");
        if (m_driver_states_->ego_race_position < m_driver_states_->secondary_vehicle_race_position)
        {
          //car1 is defending (ahead) and car2 is attacking (behind)
          set_car1_speedfactor(0.75);
          set_car2_speedfactor(1.0);
          m_current_state_.attacking_car_name = m_car2_name_;
          m_current_state_.attacking_car_index = m_driver_states_->secondary_vehicle_index;
          m_current_state_.defending_car_name = m_car1_name_;
          m_current_state_.defending_car_index = m_driver_states_->ego_vehicle_index;
        }
        else
        {
          set_car2_speedfactor(0.75);
          set_car1_speedfactor(1.0);
          m_current_state_.attacking_car_name = m_car1_name_;
          m_current_state_.attacking_car_index = m_driver_states_->ego_vehicle_index;
          m_current_state_.defending_car_name = m_car2_name_;
          m_current_state_.defending_car_index = m_driver_states_->secondary_vehicle_index;
        }
        m_current_state_.description=m_current_state_.attacking_car_name + " is overtaking "   + m_current_state_.defending_car_name;
        m_current_state_.current_state=deepracing_msgs::msg::RaceSupervisorState::STATE_OVERTAKE;
        RCLCPP_DEBUG(this->get_logger(), "transition to state: Overtake");
      }
    }
    void handle_overtaking()
    {     
      double offset = m_driver_states_->ego_total_distance - m_driver_states_->secondary_vehicle_total_distance;
      bool player1_ahead = m_driver_states_->ego_race_position < m_driver_states_->secondary_vehicle_race_position, positions_flipped;
      if(m_current_state_.attacking_car_index == m_driver_states_->secondary_vehicle_index)
      {
        offset*=-1.0;
        positions_flipped = !player1_ahead;
      }
      else
      {
        positions_flipped = player1_ahead;
      }
      if (positions_flipped && offset>30.0)
      {
        //overtake is done, transition back to follow raceline and swap roles.
        set_car1_speedfactor(1.0);
        set_car2_speedfactor(1.0);
        std::string temp(m_current_state_.defending_car_name);
        m_current_state_.defending_car_name=m_current_state_.attacking_car_name;
        m_current_state_.attacking_car_name=temp;

        uint8_t temp_idx(m_current_state_.defending_car_index);
        m_current_state_.defending_car_index=m_current_state_.attacking_car_index;
        m_current_state_.attacking_car_index=temp_idx;

        m_current_state_.description = "Following racelines at full speed. " + m_current_state_.defending_car_name + " is defending and " + m_current_state_.attacking_car_name + " is attacking.";
        RCLCPP_DEBUG(this->get_logger(), "transition to state: Following racelines");
        double deltat = 10.0*(1.0 + m_rng_.uniform01());
        m_time_of_next_overtake_ = this->get_clock()->now() + rclcpp::Duration::from_seconds(deltat);
        m_current_state_.current_state=deepracing_msgs::msg::RaceSupervisorState::STATE_FOLLOWING_RACELINES;
      }
    }
    void handle_uninitialized()
    {
      RCLCPP_DEBUG(this->get_logger(), "handle_uninitialized");
      
      if(!m_car1_param_setter_->wait_for_service(std::chrono::milliseconds(100)))
      {
        RCLCPP_ERROR(this->get_logger(), "car2 params service not available.");
        return;
      } 
      if(!m_car2_param_setter_->wait_for_service(std::chrono::milliseconds(100)))
      {
        RCLCPP_ERROR(this->get_logger(), "car2 params service not available.");
        return;
      }
      if(!m_car1_raceline_getter_->wait_for_service(std::chrono::milliseconds(100)))
      {
        RCLCPP_ERROR(this->get_logger(), "car1 raceline getter not available.");
        return;
      }
      if(!m_car1_raceline_setter_->wait_for_service(std::chrono::milliseconds(100)))
      {
        RCLCPP_ERROR(this->get_logger(), "car1 raceline setter not available.");
        return;
      }
      if(!m_car2_raceline_getter_->wait_for_service(std::chrono::milliseconds(100)))
      {
        RCLCPP_ERROR(this->get_logger(), "car2 raceline getter not available.");
        return;
      }
      if(!m_car2_raceline_setter_->wait_for_service(std::chrono::milliseconds(100)))
      {
        RCLCPP_ERROR(this->get_logger(), "car2 raceline setter not available.");
        return;
      }
      if (!m_driver_states_)
      {
        RCLCPP_ERROR(this->get_logger(), "No driver state data received");
        return;
      }
      if (m_driver_states_->ego_race_position < m_driver_states_->secondary_vehicle_race_position)
      {
        //car1 is defending (ahead) and car2 is attacking (behind)
        m_current_state_.defending_car_index=m_driver_states_->ego_vehicle_index; 
        m_current_state_.attacking_car_index=m_driver_states_->secondary_vehicle_index; 
        m_current_state_.defending_car_name=m_car1_name_;
        m_current_state_.attacking_car_name=m_car2_name_;
      }
      else
      {
        //car2 is defending (ahead) and car1 is attacking (behind)
        m_current_state_.defending_car_index=m_driver_states_->secondary_vehicle_index; 
        m_current_state_.attacking_car_index=m_driver_states_->ego_vehicle_index; 
        m_current_state_.defending_car_name=m_car2_name_;
        m_current_state_.attacking_car_name=m_car1_name_;
      }
      m_current_state_.description = "Following racelines at full speed. " + m_current_state_.defending_car_name + " is defending and " + m_current_state_.attacking_car_name + " is attacking.";
      RCLCPP_DEBUG(this->get_logger(), "transition to state: Following racelines");
      set_car1_speedfactor(1.0);
      set_car2_speedfactor(1.0);
      double deltat = 10.0 + 10.0*m_rng_.uniform01();
      m_time_of_next_overtake_ = this->get_clock()->now() + rclcpp::Duration::from_seconds(deltat);
      m_current_state_.current_state = deepracing_msgs::msg::RaceSupervisorState::STATE_FOLLOWING_RACELINES;
    }

    void states_CB_(const deepracing_msgs::msg::DriverStates::SharedPtr car_states)
    {
      RCLCPP_DEBUG(this->get_logger(), "Got some driver state data");
      m_driver_states_=car_states;
    }

    rclcpp::Time m_time_of_next_overtake_;
    random_numbers::RandomNumberGenerator m_rng_;
    double m_frequency_;
    rclcpp::TimerBase::SharedPtr m_timer_;
    deepracing_msgs::msg::RaceSupervisorState m_current_state_;
    deepracing_msgs::msg::DriverStates::ConstSharedPtr m_driver_states_;
    rclcpp::Subscription<deepracing_msgs::msg::DriverStates>::SharedPtr m_driver_states_listener_;
    rclcpp::Publisher<deepracing_msgs::msg::RaceSupervisorState>::SharedPtr m_current_state_publisher_;

    std::string m_car1_name_;
    rclcpp::Client<deepracing_msgs::srv::SetRaceline>::SharedPtr m_car1_raceline_setter_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr m_car1_param_setter_;
    rclcpp::Client<deepracing_msgs::srv::GetRaceline>::SharedPtr m_car1_raceline_getter_;
    int64_t m_car1_index_;

    std::string m_car2_name_;
    rclcpp::Client<deepracing_msgs::srv::SetRaceline>::SharedPtr m_car2_raceline_setter_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr m_car2_param_setter_;
    rclcpp::Client<deepracing_msgs::srv::GetRaceline>::SharedPtr m_car2_raceline_getter_;
    int64_t m_car2_index_;

};
int main(int argc, char *argv[]) {
  rclcpp::init(argc,argv);
  rclcpp::NodeOptions opshinz;
  opshinz = opshinz.allow_undeclared_parameters(true);
  std::shared_ptr<RaceSupervisorNode> node(new RaceSupervisorNode(opshinz));
  int num_threads_ = node->declare_parameter<int>("num_threads", 0), num_threads;
  if (num_threads_<=0)
  {
    num_threads = 0;
  }
  else
  {
    num_threads = num_threads_;
  }
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor( new rclcpp::executors::MultiThreadedExecutor(rclcpp::ExecutorOptions(), num_threads) );
  executor->add_node(node);
  node->start();
  executor->spin();
}