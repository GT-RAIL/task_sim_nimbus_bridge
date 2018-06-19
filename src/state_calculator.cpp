#include "task_sim_nimbus_bridge/state_calculator.h"

using namespace std;

StateCalculator::StateCalculator() : pn("~")
{
//  state_subscriber = n.subscribe("state_calculator/state", 1, &RobotExecutor::stateCallback, this);
  state_publisher = pn.advertise<task_sim::State>("state", 1, this);

//  cartesian_path_client = n.serviceClient<rail_manipulation_msgs::CartesianPath>("nimbus_moveit/cartesian_path");
  state_server = pn.advertiseService("calculate_state", &StateCalculator::calculateStateCallback, this);
}

bool StateCalculator::calculateStateCallback(task_sim::QueryState::Request &req, task_sim::QueryState::Response &res)
{
  // TODO: Calculate full state using a combination of depth segmentation, AR tags, and robot tfs
  res.state = state;

  state_publisher.publish(state);

  return true;
}
