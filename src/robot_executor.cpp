#include "task_sim_nimbus_bridge/robot_executor.h"

using namespace std;

RobotExecutor::RobotExecutor() : pn("~"),
                                 arm_client("nimbus_moveit/common_actions/arm_action"),
                                 gripper_client("gripper_actions/gripper_manipulation"),
                                 move_to_pose_client("nimbus_moveit/move_to_pose"),
                                 primitive_client("nimbus_moveit/primitive_action"),
                                 grasp_client("nimbus_moveit/common_actions/pickup")
{
  state_subscriber = n.subscribe("state_calculator_node/state", 1, &RobotExecutor::stateCallback, this);
  recognized_objects_susbcriber =
      n.subscribe("state_calculator_node/recognized_objects", 1, &RobotExecutor::itemsCallback, this);

  cartesian_path_client = n.serviceClient<rail_manipulation_msgs::CartesianPath>("nimbus_moveit/cartesian_path");
  suggest_grasps_client = n.serviceClient<fetch_grasp_suggestion::SuggestGrasps>("suggester/suggest_grasps");
  execute_server = pn.advertiseService("execute_action", &RobotExecutor::executeCallback, this);
}

void RobotExecutor::stateCallback(const task_sim::State &msg)
{
  state = msg;
}

void RobotExecutor::itemsCallback(const rail_manipulation_msgs::SegmentedObjectList &msg)
{
  items = msg;
}

bool RobotExecutor::executeCallback(task_sim::Execute::Request &req, task_sim::Execute::Response &res)
{
  switch(req.action.action_type)
  {
    case task_sim::Action::GRASP:
    {
      if (req.action.object == "apple" || req.action.object == "banana" || req.action.object == "carrot"
          || req.action.object == "daikon")
      {
        int item_index = -1;
        for (unsigned int i = 0; i < items.objects.size(); i ++)
        {
          if (items.objects[i].name == req.action.object)
          {
            item_index = i;
            break;
          }
        }

        if (item_index == -1)
        {
          ROS_INFO("Grasp target item is not currently seen by the perception system, cannot perform grasp.");
          return false;
        }

        fetch_grasp_suggestion::SuggestGrasps grasp_suggestions;
        grasp_suggestions.request.cloud = items.objects[item_index].point_cloud;
        if (!suggest_grasps_client.call(grasp_suggestions))
        {
          ROS_INFO("Could not call grasp suggestion service.");
          return false;
        }
        if (grasp_suggestions.response.grasp_list.poses.empty())
        {
          ROS_INFO("No grasps calculated!");
          return false;
        }

        for (unsigned int i = 0; i < grasp_suggestions.response.grasp_list.poses.size(); i ++)
        {
          if (i >= 3)
          {
            return false;
          }

          ROS_INFO("Attempting grasp %d", (int)i);

          //rotate grasps 90 degrees due to inconsistency with coordinate frames between fetch and nimbus
          geometry_msgs::PoseStamped grasp_pose;
          grasp_pose.header = grasp_suggestions.response.grasp_list.header;
          grasp_pose.pose = grasp_suggestions.response.grasp_list.poses[i];
          tf::Quaternion r = tf::createQuaternionFromRPY(M_PI_2, 0, 0);
          tf::Quaternion q;
          tf::quaternionMsgToTF(grasp_pose.pose.orientation, q);
          tf::quaternionTFToMsg((q*r).normalize(), grasp_pose.pose.orientation);

          //open gripper
          rail_manipulation_msgs::GripperGoal open_goal;
          open_goal.close = false;
          gripper_client.sendGoal(open_goal);
          gripper_client.waitForResult(ros::Duration(10.0));

          //execute grasp
          rail_manipulation_msgs::PickupGoal pickup_goal;
          pickup_goal.attachObject = false;
          pickup_goal.lift = false;
          pickup_goal.verify = false;
          pickup_goal.pose = grasp_pose;

          grasp_client.sendGoal(pickup_goal);
          grasp_client.waitForResult(ros::Duration(40));
          rail_manipulation_msgs::PickupResultConstPtr pickup_result = grasp_client.getResult();
          if (pickup_result->executionSuccess)
          {
            return pickup_result->success;
          }
        }
      }
      else if (req.action.object == "drawer")
      {
        //TODO: implement grasp for drawer and lid

        /* table_base_link to nimbus_ee_link:
         * - Translation: [-0.254, 0.616, 0.218]
           - Rotation: in Quaternion [-0.017, -0.008, 1.000, -0.006]
             in RPY (radian) [-0.016, 0.033, -3.129]
             in RPY (degree) [-0.910, 1.919, -179.284]

           drawer_position:
            x: -0.451086260308
            y: 0.641136275407
            theta: 0.0
         */

        //open gripper
        rail_manipulation_msgs::GripperGoal open_goal;
        open_goal.close = false;
        gripper_client.sendGoal(open_goal);
        gripper_client.waitForResult(ros::Duration(10.0));

        geometry_msgs::PoseStamped handle_pose;
        handle_pose.header.frame_id = "table_base_link";
        handle_pose.pose.position.x = state.drawer_position.x + state.drawer_opening + 0.185;
        handle_pose.pose.position.y = state.drawer_position.y;
        handle_pose.pose.position.z = .218;
        handle_pose.pose.orientation.x = -0.017;
        handle_pose.pose.orientation.y = -.008;
        handle_pose.pose.orientation.z = 1.0;
        handle_pose.pose.orientation.w = -.006;

        rail_manipulation_msgs::PickupGoal grasp_goal;
        grasp_goal.attachObject = false;
        grasp_goal.lift = false;
        grasp_goal.verify = false;
        grasp_goal.pose = handle_pose;

        grasp_client.sendGoal(grasp_goal);
        grasp_client.waitForResult(ros::Duration(40));
        rail_manipulation_msgs::PickupResultConstPtr grasp_result = grasp_client.getResult();
        if (grasp_result->executionSuccess)
        {
          return grasp_result->success;
        }
      }
      else if (req.action.object == "lid")
      {
        //TODO: implement grasp for lid

      }
      else
      {
        ROS_INFO("Not a graspable object!");
        return false;
      }
      break;
    }

    case task_sim::Action::PLACE:
    {
      geometry_msgs::PoseStamped place_pose;
      place_pose.header.frame_id = "table_base_link";

      tf::StampedTransform ee_transform;
      tf_listener.lookupTransform("nimbus_ee_link", "table_base_link", ros::Time(0), ee_transform);
      tf::quaternionTFToMsg(ee_transform.getRotation(), place_pose.pose.orientation);

      // TODO: Implement place
      if (req.action.object == "drawer")
      {

      }
      else if (req.action.object == "stack")
      {
        place_pose.pose.position.x = state.drawer_position.x;
        place_pose.pose.position.y = state.drawer_position.y;
        place_pose.pose.position.z = 0; // TODO: measure and change this

      }
      else if (req.action.object == "box")
      {
        //TODO
        return false;
      }
      else if (req.action.object == "lid")
      {
        //TODO
        return false;
      }
      else
      {
        //TODO: Implement place on table
        return false;
      }
      rail_manipulation_msgs::MoveToPoseGoal approachAngleGoal;
      approachAngleGoal.pose = place_pose;
      approachAngleGoal.planningTime = 3.0;
      move_to_pose_client.sendGoal(approachAngleGoal);
      move_to_pose_client.waitForResult(ros::Duration(30.0));
      if (!move_to_pose_client.getResult()->success)
      {
        ROS_INFO("Could not move to place pose.");
        return false;
      }

      rail_manipulation_msgs::GripperGoal open_goal;
      open_goal.close = false;
      gripper_client.sendGoal(open_goal);
      gripper_client.waitForResult(ros::Duration(10.0));

      return true;
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

      float dst = 0.2;
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
      raise_goal.distance = 0.2;
      primitive_client.sendGoal(raise_goal);
      primitive_client.waitForResult(ros::Duration(5.0));
      return primitive_client.getResult()->completion > 0;
    }

    case task_sim::Action::LOWER_ARM:
    {
      rail_manipulation_msgs::PrimitiveGoal lower_goal;
      lower_goal.primitive_type = rail_manipulation_msgs::PrimitiveGoal::TRANSLATION;
      lower_goal.axis = rail_manipulation_msgs::PrimitiveGoal::Z_AXIS;
      lower_goal.distance = -0.2;
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
