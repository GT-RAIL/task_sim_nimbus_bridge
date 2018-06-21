#ifndef OBJECT_DATA_COLLECTOR_H_
#define OBJECT_DATA_COLLECTOR_H_

//ROS
#include <pcl_ros/point_cloud.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>

//PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

//C++
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <signal.h>
#include <stdio.h>
#include <termios.h>

#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_D 0x64
#define KEYCODE_ENTER 0x0D

class ObjectDataCollector
{

public:

  /**
   * \brief Constructor
   */
  ObjectDataCollector();

  /*!
   * \brief Monitors the keyboard
   */
  void loop();

private:
  void newDataCallback(const rail_manipulation_msgs::SegmentedObjectList::ConstPtr &newData);

  void showObject(unsigned int index);

  ros::NodeHandle n, pnh;

  ros::Subscriber newDataSubscriber;
  ros::Publisher currentObjectPublisher;

  rail_manipulation_msgs::SegmentedObjectList objects;
  unsigned int index;

  boost::mutex objectMutex;

  // captured data vectors
  std::vector<float> dims;
  Eigen::Vector3f lab;

};

//convert from RGB color space to CIELAB color space, taken and adapted from pcl/registration/gicp6d
Eigen::Vector3f RGB2Lab (const Eigen::Vector3f& colorRGB);

/*!
 * \brief A function to close ROS and exit the program.
 *
 * \param sig The signal value.
 */
void shutdown(int sig);

#endif
