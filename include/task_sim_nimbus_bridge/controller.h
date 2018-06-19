#ifndef TASK_SIM_NIMBUS_BRIDGE_CONTROLLER_H_
#define TASK_SIM_NIMBUS_BRIDGE_CONTROLLER_H_

#include <ros/ros.h>
#include <task_sim/QueryState.h>
#include <task_sim/State.h>
#include <task_sim/Execute.h>

class Controller
{
public:
  Controller();

private:
  ros::NodeHandle n, pn;

  ros::ServiceClient execute_client;
  ros::ServiceClient query_state_client;
  ros::ServiceServer execute_server;

  bool executeCallback(task_sim::Execute::Request &req, task_sim::Execute::Response &res);
};

#endif //TASK_SIM_NIMBUS_BRIDGE_CONTROLLER_H_