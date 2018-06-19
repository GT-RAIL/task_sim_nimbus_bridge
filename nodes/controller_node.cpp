#include "task_sim_nimbus_bridge/controller.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_node");
  Controller c;

  ros::spin();

  return EXIT_SUCCESS;
}
