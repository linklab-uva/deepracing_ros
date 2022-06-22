#include <rclcpp/rclcpp.hpp>
#include <deepracing_msgs/msg/timestamped_packet_car_telemetry_data.hpp>
#include <deepracing_msgs/msg/timestamped_packet_car_status_data.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <deepracing_msgs/msg/race_supervisor_state.hpp>
#include <deepracing_msgs/msg/driver_states.hpp>
#include <deepracing_msgs/srv/set_raceline.hpp>
#include <deepracing_msgs/srv/get_raceline.hpp>
#include <chrono>

using namespace std::chrono_literals;

class RaceSupervisorNode : public rclcpp::Node 
{

  public:
    RaceSupervisorNode( const rclcpp::NodeOptions & options )
     : rclcpp::Node("race_supervisor", options)
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
      std::vector<std::string> car_names = declare_parameter<std::vector<std::string>>(car_names_descriptor.name, car_names_descriptor);
      m_car1_name_ = car_names[0];

      std::string car1_state_topic = "/" + m_car1_name_ + "/driver_states";
      m_car1_states_listener_ = create_subscription<deepracing_msgs::msg::DriverStates>(car1_state_topic, rclcpp::QoS{1}, std::bind(&RaceSupervisorNode::car1_states_CB_, this, std::placeholders::_1));
      std::string car1_get_raceline_service = "/" + m_car1_name_ + "/get_raceline";
      m_car1_raceline_getter_ = create_client<deepracing_msgs::srv::GetRaceline>(car1_get_raceline_service);
      std::string car1_set_raceline_service = "/" + m_car1_name_ + "/set_raceline";
      m_car1_raceline_setter_ = create_client<deepracing_msgs::srv::SetRaceline>(car1_set_raceline_service);
      std::string car1_set_params_service = "/" + m_car1_name_ + "/set_parameters";
      m_car1_param_setter_ = create_client<rcl_interfaces::srv::SetParameters>(car1_set_params_service);

      m_car2_name_ = car_names[1];
      std::string car2_state_topic = "/" + m_car2_name_ + "/driver_states";
      m_car2_states_listener_ = create_subscription<deepracing_msgs::msg::DriverStates>(car2_state_topic, rclcpp::QoS{1}, std::bind(&RaceSupervisorNode::car2_states_CB_, this, std::placeholders::_1));
      std::string car2_get_raceline_service = "/" + m_car2_name_ + "/get_raceline";
      m_car2_raceline_getter_ = create_client<deepracing_msgs::srv::GetRaceline>(car2_get_raceline_service);
      std::string car2_set_raceline_service = "/" + m_car2_name_ + "/set_raceline";
      m_car2_raceline_setter_ = create_client<deepracing_msgs::srv::SetRaceline>(car2_set_raceline_service);
      std::string car2_set_params_service = "/" + m_car2_name_ + "/set_parameters";
      m_car2_param_setter_ = create_client<rcl_interfaces::srv::SetParameters>(car2_set_params_service);

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
      m_current_state_publisher_->publish(m_current_state_);
    }
    void handle_uninitialized()
    {
      RCLCPP_DEBUG(this->get_logger(), "handle_uninitialized");
      if (!m_car1_states_ || !m_car2_states_)
      {
        RCLCPP_ERROR(this->get_logger(), "No driver state data received");
        return;
      }
      if (m_car1_states_->ego_race_position < m_car2_states_->ego_race_position)
      {
        //car1 is defending (ahead) and car2 is attacking (behind)
        m_current_state_.defending_car_index=m_car1_states_->ego_vehicle_index; 
        m_current_state_.attacking_car_index=m_car2_states_->ego_vehicle_index; 
        m_current_state_.defending_car_name=m_car1_name_;
        m_current_state_.attacking_car_name=m_car2_name_;
      }
      else
      {
        //car2 is defending (ahead) and car1 is attacking (behind)
        m_current_state_.defending_car_index=m_car2_states_->ego_vehicle_index; 
        m_current_state_.attacking_car_index=m_car1_states_->ego_vehicle_index; 
        m_current_state_.defending_car_name=m_car2_name_;
        m_current_state_.attacking_car_name=m_car1_name_;
      }
      m_current_state_.description = "Following racelines at full speed. " + m_current_state_.defending_car_name + " is defending and " + m_current_state_.attacking_car_name + " is attacking.";
      RCLCPP_DEBUG(this->get_logger(), "transition to state: Following racelines");
      m_current_state_.current_state = deepracing_msgs::msg::RaceSupervisorState::STATE_FOLLOWING_RACELINES;
    }
    void car1_states_CB_(const deepracing_msgs::msg::DriverStates::ConstSharedPtr& car1_states)
    {
      RCLCPP_DEBUG(this->get_logger(), "Got some car1 state data");
      m_car1_states_=car1_states;
    }
    void car2_states_CB_(const deepracing_msgs::msg::DriverStates::ConstSharedPtr& car2_states)
    {
      RCLCPP_DEBUG(this->get_logger(), "Got some car2 state data");
      m_car2_states_=car2_states;
    }
    double m_frequency_;
    rclcpp::TimerBase::SharedPtr m_timer_;
    deepracing_msgs::msg::RaceSupervisorState m_current_state_;
    rclcpp::Publisher<deepracing_msgs::msg::RaceSupervisorState>::SharedPtr m_current_state_publisher_;

    deepracing_msgs::msg::DriverStates::ConstSharedPtr m_car1_states_;
    std::string m_car1_name_;
    rclcpp::Subscription<deepracing_msgs::msg::DriverStates>::SharedPtr m_car1_states_listener_;
    rclcpp::Client<deepracing_msgs::srv::SetRaceline>::SharedPtr m_car1_raceline_setter_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr m_car1_param_setter_;
    rclcpp::Client<deepracing_msgs::srv::GetRaceline>::SharedPtr m_car1_raceline_getter_;

    deepracing_msgs::msg::DriverStates::ConstSharedPtr m_car2_states_;
    std::string m_car2_name_;
    rclcpp::Subscription<deepracing_msgs::msg::DriverStates>::SharedPtr m_car2_states_listener_;
    rclcpp::Client<deepracing_msgs::srv::SetRaceline>::SharedPtr m_car2_raceline_setter_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr m_car2_param_setter_;
    rclcpp::Client<deepracing_msgs::srv::GetRaceline>::SharedPtr m_car2_raceline_getter_;

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