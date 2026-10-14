#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

class FalcoTelemetryNode : public rclcpp::Node
{
public:
  FalcoTelemetryNode() : Node("falco_telemetry_node")
  {
    // SensorDataQoS usa best-effort, adatto ai topic di telemetria PX4.
    const auto qos = rclcpp::SensorDataQoS();

    position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos,
      [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        position_received_ = true;
        xy_valid_ = msg->xy_valid;
        z_valid_ = msg->z_valid;
        x_ = msg->x;
        y_ = msg->y;
        z_ = msg->z;
        vx_ = msg->vx;
        vy_ = msg->vy;
        vz_ = msg->vz;
        heading_ = msg->heading;
      });

    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos,
      [this](px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        attitude_received_ = true;
        q_ = msg->q;
      });

    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", qos,
      [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        status_received_ = true;
        armed_ =
          msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        nav_state_ = msg->nav_state;
        failsafe_ = msg->failsafe;
      });

    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() { report_telemetry(); });

    RCLCPP_INFO(
      get_logger(),
      "FALCO telemetry node avviato. Modalita: sola lettura; nessun comando di volo.");
    RCLCPP_INFO(
      get_logger(),
      "In ascolto: /fmu/out/vehicle_local_position, "
      "/fmu/out/vehicle_attitude, /fmu/out/vehicle_status");
  }

private:
  static std::string navigation_state_name(uint8_t state)
  {
    switch (state) {
      case 0: return "MANUAL";
      case 1: return "ALTCTL";
      case 2: return "POSCTL";
      case 3: return "AUTO_MISSION";
      case 4: return "AUTO_LOITER";
      case 5: return "AUTO_RTL";
      case 10: return "ACRO";
      case 12: return "DESCEND";
      case 13: return "TERMINATION";
      case 14: return "OFFBOARD";
      case 15: return "STAB";
      case 17: return "AUTO_TAKEOFF";
      case 18: return "AUTO_LAND";
      default: return "OTHER";
    }
  }

  void report_telemetry()
  {
    if (!position_received_ && !attitude_received_ && !status_received_) {
      RCLCPP_WARN(
        get_logger(),
        "Nessuna telemetria ricevuta. Verificare publisher PX4, topic e trasporto ROS 2.");
      return;
    }

    RCLCPP_INFO(
      get_logger(), "Ricezione: posizione=%s, orientamento=%s, stato=%s",
      position_received_ ? "SI" : "NO",
      attitude_received_ ? "SI" : "NO",
      status_received_ ? "SI" : "NO");

    if (position_received_) {
      RCLCPP_INFO(
        get_logger(),
        "NED [m]: x=%.2f y=%.2f z=%.2f | "
        "velocita [m/s]: vx=%.2f vy=%.2f vz=%.2f | heading=%.3f rad | "
        "validita: xy=%s z=%s",
        x_, y_, z_, vx_, vy_, vz_, heading_,
        xy_valid_ ? "SI" : "NO", z_valid_ ? "SI" : "NO");
    }

    if (attitude_received_) {
      RCLCPP_INFO(
        get_logger(), "Quaternione [w,x,y,z]: [%.3f, %.3f, %.3f, %.3f]",
        q_[0], q_[1], q_[2], q_[3]);
    }

    if (status_received_) {
      RCLCPP_INFO(
        get_logger(), "Stato: %s | modalita=%s (%u) | failsafe=%s",
        armed_ ? "ARMATO" : "DISARMATO",
        navigation_state_name(nav_state_).c_str(),
        static_cast<unsigned int>(nav_state_),
        failsafe_ ? "SI" : "NO");
    }
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool position_received_{false};
  bool attitude_received_{false};
  bool status_received_{false};

  bool xy_valid_{false};
  bool z_valid_{false};
  float x_{0.0F}, y_{0.0F}, z_{0.0F};
  float vx_{0.0F}, vy_{0.0F}, vz_{0.0F};
  float heading_{0.0F};
  std::array<float, 4> q_{};

  bool armed_{false};
  uint8_t nav_state_{0};
  bool failsafe_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FalcoTelemetryNode>());
  rclcpp::shutdown();
  return 0;
}
