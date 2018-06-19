#include "task_sim_nimbus_bridge/robot_executor.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_executor_node");
  RobotExecutor re;

  ros::spin();

  return EXIT_SUCCESS;
}
