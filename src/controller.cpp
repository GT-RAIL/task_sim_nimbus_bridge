#include "task_sim_nimbus_bridge/controller.h"

using namespace std;

Controller::Controller() : pn("~")
{
  execute_client = n.serviceClient<task_sim::Execute>("robot_executor_node/execute_action");
  query_state_client = n.serviceClient<task_sim::QueryState>("state_calculator_node/calculate_state");
  update_state_client = n.serviceClient<task_sim_nimbus_bridge::UpdateState>("state_calculator_node/update_state");
  execute_server = pn.advertiseService("execute", &Controller::executeCallback, this);
}

bool Controller::executeCallback(task_sim::Execute::Request &req, task_sim::Execute::Response &res)
{
  task_sim::Execute action_executor;
  action_executor.request.action = req.action;

  // execute given action
  bool execution_result = execute_client.call(action_executor);

  // give everything some time to settle
  ros::Duration(2.0).sleep();

  if (!calculateState())
    return false;

  if (execution_result)
  {
    // action-specific state updates on action success
    if (req.action.action_type == task_sim::Action::GRASP)
    {
      if (req.action.object == "drawer")
      {
        state.object_in_gripper = "drawer";
      }
      else if (req.action.object == "lid")
      {
        state.object_in_gripper = "lid";
      }
      else
      {
        for (unsigned int i = 0; i < state.objects.size(); i ++)
        {
          if (req.action.object == state.objects[i].name)
          {
            state.object_in_gripper = req.action.object;
            state.objects[i].in_gripper = true;
            break;
          }
        }
      }
    }
    else if (req.action.action_type == task_sim::Action::PLACE)
    {
      if (state.object_in_gripper != "")
      {
        for (unsigned int i = 0; i < state.objects.size(); i ++)
        {
          if (req.action.object == state.objects[i].name)
          {
            state.objects[i].position.x = state.gripper_position.x;
            state.objects[i].position.y = state.gripper_position.y;
            state.objects[i].position.z = state.gripper_position.z - 0.05;
            state.objects[i].in_gripper = false;
            break;
          }
        }

        state.object_in_gripper = "";
      }
    }
    else if (req.action.action_type == task_sim::Action::MOVE_ARM ||
        req.action.action_type == task_sim::Action::RAISE_ARM ||
        req.action.action_type == task_sim::Action::LOWER_ARM ||
        req.action.action_type == task_sim::Action::RESET_ARM)
    {
      if (state.object_in_gripper != "")
      {
        for (unsigned int i = 0; i < state.objects.size(); i ++)
        {
          if (req.action.object == state.objects[i].name)
          {
            state.objects[i].position.x = state.gripper_position.x;
            state.objects[i].position.y = state.gripper_position.y;
            state.objects[i].position.z = state.gripper_position.z;
            break;
          }
        }
      }
    }
    else if (req.action.action_type == task_sim::Action::OPEN_GRIPPER)
    {
      if (state.object_in_gripper != "")
      {
        for (unsigned int i = 0; i < state.objects.size(); i ++)
        {
          if (state.object_in_gripper == state.objects[i].name)
          {
            state.objects[i].in_gripper = false;
            break;
          }
        }
      }
      state.object_in_gripper = "";
    }
    else if (req.action.action_type == task_sim::Action::CLOSE_GRIPPER)
    {
      // TODO: find closest object and set that as object in gripper (maybe move this into state calculator somehow?)
    }

    task_sim_nimbus_bridge::UpdateState update;
    update.request.state = state;
    if (!update_state_client.call(update))
    {
      ROS_INFO("Could not call state updater");
    }

    if (!calculateState())
      return false;

    // check for incorrect assignment to object_in_gripper
    if (state.object_in_gripper != "")
    {
      bool change = false;
      for (unsigned int i = 0; i < state.objects.size(); i ++)
      {
        if (state.object_in_gripper == state.objects[i].name)
        {
          if (sqrt(pow(state.gripper_position.x - state.objects[i].position.x, 2) +
                       pow(state.gripper_position.y - state.objects[i].position.y, 2) +
                       pow(state.gripper_position.z - state.objects[i].position.z, 2)) > 0.2)
          {
            state.object_in_gripper = "";
            state.objects[i].in_gripper = false;
            change = true;
          }
          break;
        }
      }
      if (change)
      {
        update.request.state = state;
        if (!update_state_client.call(update))
        {
          ROS_INFO("Could not call state updater");
        }
      }
    }
  }

  res.state = state;

  return true;
}

bool Controller::calculateState()
{
  task_sim::QueryState queryState;
  if (!query_state_client.call(queryState))
  {
    ROS_INFO("Could not call state calculator.");
    return false;
  }
  state = queryState.response.state;
  return true;
}


