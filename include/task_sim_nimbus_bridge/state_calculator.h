#ifndef TASK_SIM_NIMBUS_BRIDGE_STATE_CALCULATOR_H_
#define TASK_SIM_NIMBUS_BRIDGE_STATE_CALCULATOR_H_

//#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <task_sim/QueryState.h>
#include <task_sim/State.h>
#include <tf/transform_listener.h>

class StateCalculator
{
public:
  StateCalculator();

private:
  ros::NodeHandle n, pn;

//  ros::Subscriber state_subscriber;
  ros::Publisher state_publisher;

  ros::ServiceServer state_server;

  tf::TransformListener tf_listener;

  task_sim::State state;

//  void stateCallback(const task_sim::State &msg);

  bool calculateStateCallback(task_sim::QueryState::Request &req, task_sim::QueryState::Response &res);
};

#endif //TASK_SIM_NIMBUS_BRIDGE_STATE_CALCULATOR_H_