#include "task_sim_nimbus_bridge/state_calculator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_calculator_node");
  StateCalculator sc;

  ros::spin();

  return EXIT_SUCCESS;
}
