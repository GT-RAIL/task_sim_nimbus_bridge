#ifndef TASK_SIM_NIMBUS_BRIDGE_STATE_CALCULATOR_H_
#define TASK_SIM_NIMBUS_BRIDGE_STATE_CALCULATOR_H_

//#include <actionlib/client/simple_action_client.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <boost/thread/mutex.hpp>
#include <Eigen/Core>
//#include <pcl_ros/point_cloud.h>
//#include <pcl/common/common.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <task_sim/QueryState.h>
#include <task_sim/State.h>
#include <task_sim_nimbus_bridge/Classify.h>
#include <tf/transform_listener.h>

class StateCalculator
{
public:
  StateCalculator();

private:
  ros::NodeHandle n, pn;

  ros::Subscriber ar_subscriber;
  ros::Subscriber segmented_objects_subscriber;
  ros::Publisher state_publisher;

  ros::ServiceClient segment_client;
  ros::ServiceClient classify_client;
  ros::ServiceServer state_server;

  tf::TransformListener tf_listener;

  task_sim::State state;

  geometry_msgs::PoseStamped drawer_pose;
  geometry_msgs::PoseStamped stack_pose;

  rail_manipulation_msgs::SegmentedObjectList segmented_objects;
  rail_manipulation_msgs::SegmentedObjectList recognized_objects;

  bool segmented_objects_updated;

  boost::mutex ar_mutex;

  void arCallback(const ar_track_alvar_msgs::AlvarMarkers &msg);

  void segmentedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &msg);

  bool calculateStateCallback(task_sim::QueryState::Request &req, task_sim::QueryState::Response &res);
};

Eigen::Vector3f RGB2Lab (const Eigen::Vector3f& colorRGB);

#endif //TASK_SIM_NIMBUS_BRIDGE_STATE_CALCULATOR_H_