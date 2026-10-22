#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ur30_ibvs/ibvs_state_machine.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ur30_ibvs::IbvsStateMachine>());
  rclcpp::shutdown();
  return 0;
}
