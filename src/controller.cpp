#include "task_sim_nimbus_bridge/controller.h"

using namespace std;

Controller::Controller() : pn("~")
{
  execute_client = n.serviceClient<task_sim::Execute>("robot_executor_node/execute_action");
  query_state_client = n.serviceClient<task_sim::QueryState>("state_calculator_node/calculate_state");
  execute_server = pn.advertiseService("execute", &Controller::executeCallback, this);
}

bool Controller::executeCallback(task_sim::Execute::Request &req, task_sim::Execute::Response &res)
{
  task_sim::Execute action_executor;
  action_executor.request.action = req.action;

  // execute given action
  if (!execute_client.call(action_executor))
  {
    ROS_INFO("Could not call robot action executor.");
    return false;
  }

  // give everything some time to settle
  ros::Duration(3.0).sleep();

  // calculate and return new state
  task_sim::QueryState queryState;
  if (!query_state_client.call(queryState))
  {
    ROS_INFO("Could not call state calculator.");
    return false;
  }
  res.state = queryState.response.state;

  return true;
}
