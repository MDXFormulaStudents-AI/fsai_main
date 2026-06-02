// main.cpp — entry point for the fsai_vehicle_interface node
//
// Creates a single VehicleInterfaceNode and spins it until Ctrl-C.
// The node's destructor sends 20 zero-frames to the VCU before exit.

#include <rclcpp/rclcpp.hpp>
#include "fsai_vehicle_interface/vehicle_interface_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<fsai_vehicle_interface::VehicleInterfaceNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
