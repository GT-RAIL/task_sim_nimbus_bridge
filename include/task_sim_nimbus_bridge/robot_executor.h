#ifndef TASK_SIM_NIMBUS_BRIDGE_ROBOT_EXECUTOR_H_
#define TASK_SIM_NIMBUS_BRIDGE_ROBOT_EXECUTOR_H_

#include <actionlib/client/simple_action_client.h>
#include <fetch_grasp_suggestion/SuggestGrasps.h>
#include <kinova_msgs/JointVelocity.h>
#include <rail_manipulation_msgs/ArmAction.h>
#include <rail_manipulation_msgs/CartesianPath.h>
#include <rail_manipulation_msgs/GripperAction.h>
#include <rail_manipulation_msgs/MoveToPoseAction.h>
#include <rail_manipulation_msgs/PickupAction.h>
#include <rail_manipulation_msgs/PrimitiveAction.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <ros/ros.h>
#include <task_sim/Action.h>
#include <task_sim/Execute.h>
#include <task_sim/State.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class RobotExecutor
{
public:
  RobotExecutor();

private:
  ros::NodeHandle n, pn;

  ros::Subscriber state_subscriber;
  ros::Subscriber recognized_objects_susbcriber;

  ros::Publisher pose_debug_publisher;
  ros::Publisher joint_cmd_publisher;

  ros::ServiceClient cartesian_path_client;
  ros::ServiceClient suggest_grasps_client;
  ros::ServiceServer execute_server;

  actionlib::SimpleActionClient<rail_manipulation_msgs::ArmAction> arm_client;
  actionlib::SimpleActionClient<rail_manipulation_msgs::GripperAction> gripper_client;
  actionlib::SimpleActionClient<rail_manipulation_msgs::MoveToPoseAction> move_to_pose_client;
  actionlib::SimpleActionClient<rail_manipulation_msgs::PrimitiveAction> primitive_client;
  actionlib::SimpleActionClient<rail_manipulation_msgs::PickupAction> grasp_client;

  tf::TransformListener tf_listener;

  task_sim::State state;

  rail_manipulation_msgs::SegmentedObjectList items;

  void stateCallback(const task_sim::State &msg);

  void itemsCallback(const rail_manipulation_msgs::SegmentedObjectList &msg);

  bool executeCallback(task_sim::Execute::Request &req, task_sim::Execute::Response &res);
};

#endif //TASK_SIM_NIMBUS_BRIDGE_ROBOT_EXECUTOR_H_