#include "task_sim_nimbus_bridge/robot_executor.h"

using namespace std;

RobotExecutor::RobotExecutor() : pn("~"),
                                 arm_client("nimbus_moveit/common_actions/arm_action"),
                                 gripper_client("gripper_actions/gripper_manipulation"),
                                 primitive_client("nimbus_moveit/primitive_action")
{
  state_subscriber = n.subscribe("state_calculator/state", 1, &RobotExecutor::stateCallback, this);

  cartesian_path_client = n.serviceClient<rail_manipulation_msgs::CartesianPath>("nimbus_moveit/cartesian_path");
  execute_server = pn.advertiseService("execute_action", &RobotExecutor::executeCallback, this);
}

void RobotExecutor::stateCallback(const task_sim::State &msg)
{
  state = msg;
}

bool RobotExecutor::executeCallback(task_sim::Execute::Request &req, task_sim::Execute::Response &res)
{
  switch(req.action.action_type)
  {
    case task_sim::Action::GRASP:
    {
      // TODO: Implement grasp
      break;
    }

    case task_sim::Action::PLACE:
    {
      // TODO: Implement place
      break;
    }

    case task_sim::Action::OPEN_GRIPPER:
    {
      rail_manipulation_msgs::GripperGoal open_goal;
      open_goal.close = false;
      gripper_client.sendGoal(open_goal);
      gripper_client.waitForResult(ros::Duration(10.0));
      return gripper_client.getResult()->success;
    }

    case task_sim::Action::CLOSE_GRIPPER:
    {
      rail_manipulation_msgs::GripperGoal close_goal;
      close_goal.close = true;
      gripper_client.sendGoal(close_goal);
      gripper_client.waitForResult(ros::Duration(10.0));
      return gripper_client.getResult()->success;
    }

    case task_sim::Action::MOVE_ARM:
    {
      rail_manipulation_msgs::CartesianPath path;
      path.request.avoidCollisions = false;

      tf::StampedTransform start_transform;
      tf_listener.lookupTransform("table_base_link", "nimbus_ee_link", ros::Time(0), start_transform);
      geometry_msgs::PoseStamped start_pose;
      start_pose.header.frame_id = "table_base_link";
      start_pose.header.stamp = ros::Time::now();
      start_pose.pose.position.x = start_transform.getOrigin().x();
      start_pose.pose.position.y = start_transform.getOrigin().y();
      start_pose.pose.position.z = start_transform.getOrigin().z();
      tf::quaternionTFToMsg(start_transform.getRotation(), start_pose.pose.orientation);

      geometry_msgs::PoseStamped end_pose;
      end_pose.header = start_pose.header;
      end_pose.pose = start_pose.pose;

      float dst = 0.1;
      if (req.action.object == "l")
      {
        end_pose.pose.position.x -= dst;
      }
      else if (req.action.object == "fl")
      {
        end_pose.pose.position.x -= dst;
        end_pose.pose.position.y -= dst;
      }
      else if (req.action.object == "f")
      {
        end_pose.pose.position.y -= dst;
      }
      else if (req.action.object == "fr")
      {
        end_pose.pose.position.x += dst;
        end_pose.pose.position.y -= dst;
      }
      else if (req.action.object == "r")
      {
        end_pose.pose.position.x += dst;
      }
      else if (req.action.object == "br")
      {
        end_pose.pose.position.x += dst;
        end_pose.pose.position.y += dst;
      }
      else if (req.action.object == "b")
      {
        end_pose.pose.position.y += dst;
      }
      else if (req.action.object == "bl")
      {
        end_pose.pose.position.x -= dst;
        end_pose.pose.position.y += dst;
      }
      else
      {
        bool goal_found = false;
        for (unsigned int i = 0; i < state.objects.size(); i ++)
        {
          if (state.objects[i].name == req.action.object)
          {
            end_pose.pose.position.x = state.objects[i].position.x;
            end_pose.pose.position.y = state.objects[i].position.y;
            goal_found = true;
            break;
          }
        }

        if (!goal_found)
        {
          if (req.action.object == "drawer")
          {
            end_pose.pose.position.x = state.drawer_position.x + state.drawer_opening;
            end_pose.pose.position.y = state.drawer_position.y;
          }
          else if (req.action.object == "stack")
          {
            end_pose.pose.position.x = state.drawer_position.x;
            end_pose.pose.position.y = state.drawer_position.y;
          }
          else if (req.action.object == "lid")
          {
            end_pose.pose.position.x = state.lid_position.x;
            end_pose.pose.position.y = state.lid_position.y;
          }
          else if (req.action.object == "box")
          {
            end_pose.pose.position.x = state.box_position.x;
            end_pose.pose.position.y = state.box_position.y;
          }
        }
      }

      path.request.waypoints.push_back(end_pose);
      if (!cartesian_path_client.call(path))
      {
        ROS_INFO("Could not call Cartesian path service!");
        return false;
      }

      return path.response.completion > 0;
    }

    case task_sim::Action::RAISE_ARM:
    {
      rail_manipulation_msgs::PrimitiveGoal raise_goal;
      raise_goal.primitive_type = rail_manipulation_msgs::PrimitiveGoal::TRANSLATION;
      raise_goal.axis = rail_manipulation_msgs::PrimitiveGoal::Z_AXIS;
      raise_goal.distance = 0.1;
      primitive_client.sendGoal(raise_goal);
      primitive_client.waitForResult(ros::Duration(5.0));
      return primitive_client.getResult()->completion > 0;
    }

    case task_sim::Action::LOWER_ARM:
    {
      rail_manipulation_msgs::PrimitiveGoal lower_goal;
      lower_goal.primitive_type = rail_manipulation_msgs::PrimitiveGoal::TRANSLATION;
      lower_goal.axis = rail_manipulation_msgs::PrimitiveGoal::Z_AXIS;
      lower_goal.distance = -0.1;
      primitive_client.sendGoal(lower_goal);
      primitive_client.waitForResult(ros::Duration(5.0));
      return primitive_client.getResult()->completion > 0;
    }

    case task_sim::Action::RESET_ARM:
    {
      rail_manipulation_msgs::ArmGoal reset_goal;
      reset_goal.action = rail_manipulation_msgs::ArmGoal::READY;
      arm_client.sendGoal(reset_goal);
      arm_client.waitForResult(ros::Duration(20.0));
      return arm_client.getResult()->success;
    }

    default:
      return true;
  }

  return true;
}
