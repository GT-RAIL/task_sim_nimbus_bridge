#include "task_sim_nimbus_bridge/state_calculator.h"
#include "task_sim_nimbus_bridge/bounding_box_calculator.h"

using namespace std;

StateCalculator::StateCalculator() : pn("~")
{
  segmented_objects_updated = true;

  ar_subscriber = n.subscribe("ar_pose_marker", 1, &StateCalculator::arCallback, this);
  segmented_objects_subscriber = n.subscribe("rail_segmentation/segmented_objects", 1,
                                             &StateCalculator::segmentedObjectsCallback, this);
  gripper_state_subscriber = n.subscribe("gripper/joint_states", 1, &StateCalculator::gripperStateCallback, this);
  state_publisher = pn.advertise<task_sim::State>("state", 1, this);
  recognized_objects_publisher =
      pn.advertise<rail_manipulation_msgs::SegmentedObjectList>("recognized_objects", 1, this);

  segment_client = n.serviceClient<std_srvs::Empty>("rail_segmentation/segment");
  classify_client = n.serviceClient<task_sim_nimbus_bridge::Classify>("object_classifier/classify");
  state_server = pn.advertiseService("calculate_state", &StateCalculator::calculateStateCallback, this);
  update_state_server = pn.advertiseService("update_state", &StateCalculator::updateStateCallback, this);
}

void StateCalculator::gripperStateCallback(const sensor_msgs::JointState &msg)
{
  if (!msg.position.empty())
    state.gripper_open = msg.position[0] < 0.05;
}

bool StateCalculator::calculateStateCallback(task_sim::QueryState::Request &req, task_sim::QueryState::Response &res)
{
  // segmentation updates (items)
  segmented_objects_updated = false;

  std_srvs::Empty segment;
  if (!segment_client.call(segment))
  {
    ROS_INFO("Couldn't call segmentation service.");
    return false;
  }

  ros::Time start = ros::Time::now();
  bool timeout = false;
  while (!segmented_objects_updated && !timeout)
  {
    timeout = (ros::Time::now() - start).toSec() > 10.0;
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  if (timeout)
  {
    ROS_INFO("Couldn't segment objects.");
    return false;
  }

  // AR tag updates (containers)
  {
    boost::mutex::scoped_lock lock(ar_mutex);
    state.drawer_position.x = stack_pose.pose.position.x;
    state.drawer_position.y = stack_pose.pose.position.y;
    state.drawer_opening = drawer_pose.pose.position.x - stack_pose.pose.position.x;
    if (state.drawer_opening < 0)
    {
      state.drawer_opening = 0;
    }

    state.box_position.x = box_pose.pose.position.x;
    state.box_position.y = box_pose.pose.position.y;

    // TODO: switch lid_position to use segmented/recognized objects
//    state.lid_position.x = lid_pose.pose.position.x;
//    state.lid_position.y = lid_pose.pose.position.y;
//    state.lid_position.z = lid_pose.pose.position.z;
  }

  // Robot update
  tf::StampedTransform gripper_transform;
  tf_listener.lookupTransform("table_base_link", "nimbus_ee_link", ros::Time(0), gripper_transform);
  state.gripper_position.x = gripper_transform.getOrigin().x();
  state.gripper_position.y = gripper_transform.getOrigin().y();
  state.gripper_position.z = gripper_transform.getOrigin().z();

  res.state = state;

  state_publisher.publish(state);

  return true;
}

void StateCalculator::arCallback(const ar_track_alvar_msgs::AlvarMarkers &msg)
{
  boost::mutex::scoped_lock lock(ar_mutex);

  for (unsigned int i = 0; i < msg.markers.size(); i ++)
  {
    if (msg.markers[i].id == 1)
    {
      // stack position
      geometry_msgs::PoseStamped marker_pose = msg.markers[i].pose;
      marker_pose.header.frame_id = msg.markers[i].header.frame_id;
      tf_listener.transformPose("table_base_link", ros::Time(0), marker_pose, "table_base_link", stack_pose);
      stack_pose.header.frame_id = "table_base_link";

      stack_pose.pose.position.x += 0.15;
      stack_pose.pose.position.y += 0.085;
      stack_pose.pose.position.z -= 0.08;
    }
    else if (msg.markers[i].id == 4)
    {
      // drawer position
      geometry_msgs::PoseStamped marker_pose = msg.markers[i].pose;
      marker_pose.header.frame_id = msg.markers[i].header.frame_id;
      tf_listener.transformPose("table_base_link", ros::Time(0), marker_pose, "table_base_link", drawer_pose);
      drawer_pose.header.frame_id = "table_base_link";

      drawer_pose.pose.position.x -= 0.157;
      drawer_pose.pose.position.y += 0.085;
      drawer_pose.pose.position.z += 0.025;
    }
    else if (msg.markers[i].id == 2)
    {
      // box position
      geometry_msgs::PoseStamped marker_pose = msg.markers[i].pose;
      marker_pose.header.frame_id = msg.markers[i].header.frame_id;
      tf_listener.transformPose("table_base_link", ros::Time(0), marker_pose, "table_base_link", box_pose);
      box_pose.header.frame_id = "table_base_link";

      box_pose.pose.position.x -= 0.133;
      box_pose.pose.position.y += 0;
      box_pose.pose.position.z -= 0.038;
    }
//    else if (msg.markers[i].id == 3)
//    {
//      //TODO: move lid position to segmentation/recognition
//      geometry_msgs::PoseStamped marker_pose = msg.markers[i].pose;
//      marker_pose.header.frame_id = msg.markers[i].header.frame_id;
//      marker_pose.pose.position.x -= 0.095;
//      marker_pose.pose.position.y += 0;
//      marker_pose.pose.position.z -= 0.0381;
//      tf_listener.transformPose("table_base_link", ros::Time(0), marker_pose, "table_base_link", lid_pose);
//      lid_pose.header.frame_id = "table_base_link";
//    }
  }
}

void StateCalculator::segmentedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &msg)
{
  if (msg.cleared)
  {
    return;
  }

  recognized_objects.objects.clear();

  rail_manipulation_msgs::SegmentedObjectList lid_objects;
  lid_objects.header.frame_id = msg.header.frame_id;
  lid_objects.objects.clear();
  bool lid_found = false;
  vector<string> updated;
  for (unsigned int i = 0; i < msg.objects.size(); i ++)
  {
    // calculate features for recognition
    Eigen::Vector3f rgb, lab;
    rgb[0] = msg.objects[i].marker.color.r;
    rgb[1] = msg.objects[i].marker.color.g;
    rgb[2] = msg.objects[i].marker.color.b;
    lab = RGB2Lab(rgb);

    task_sim_nimbus_bridge::BoundingBox box = BoundingBoxCalculator::computeBoundingBox(msg.objects[i].point_cloud);

    task_sim_nimbus_bridge::Classify classify;
    classify.request.features.push_back(lab[0]);
    classify.request.features.push_back(lab[1]);
    classify.request.features.push_back(lab[2]);
    if (box.dimensions.y > box.dimensions.z)
    {
      classify.request.features.push_back(box.dimensions.z);
      classify.request.features.push_back(box.dimensions.y);
    }
    else
    {
      classify.request.features.push_back(box.dimensions.y);
      classify.request.features.push_back(box.dimensions.z);
    }
    classify.request.features.push_back(box.dimensions.x);

    if (not classify_client.call(classify))
    {
      segmented_objects_updated = true;
      return;
    }

    string label = classify.response.label;
    // remap label to work with learned policy (uses fruits and vegetables...)
    if (label == "apple")
      label = "carrot";
    else if (label == "banana")
      label = "daikon";
    else if (label == "tape")
      label = "apple";
    else if (label == "marker")
      label = "banana";
    else if (label == "lid" || label == "lidbox" || label == "box")
    {
      if (label == "lid")
        lid_found = true;
      lid_objects.objects.push_back(msg.objects[i]);
      lid_objects.objects[lid_objects.objects.size() - 1].name = label;
    }

    if (label == "apple" or label == "banana" or label == "carrot" or label == "daikon")
    {
      updated.push_back(label);
      bool object_updated = false;
      for (unsigned int j = 0; j < state.objects.size(); j ++)
      {
        if (state.objects[j].name == label)
        {
          state.objects[j].position = msg.objects[i].center;
          // don't worry about in_drawer, in_box, on_lid, on_stack, as they are calculated in the relation-based state
          // in_gripper handled at higher level

          object_updated = true;
          break;
        }
      }
      if (!object_updated)
      {
        task_sim::Object item;
        item.name = label;
        item.unique_name = item.name;
        item.position = msg.objects[i].center;
        // don't worry about in_drawer, in_box, on_lid, on_stack, as they are calculated in the relation-based state
        // in_gripper handled at higher level

        state.objects.push_back(item);
      }

      recognized_objects.objects.push_back(msg.objects[i]);
      recognized_objects.objects[recognized_objects.objects.size() - 1].recognized = true;
      recognized_objects.objects[recognized_objects.objects.size() - 1].name = label;
    }
  }

  if (lid_found)
  {
    for (unsigned int i = 0; i < lid_objects.objects.size(); i ++)
    {
      if (lid_objects.objects[i].name == "lid")
      {
        recognized_objects.objects.push_back(lid_objects.objects[i]);
        state.lid_position = lid_objects.objects[i].center;

        break;
      }
    }
  }
  else if (!lid_objects.objects.empty())
  {
    for (unsigned int i = 0; i < lid_objects.objects.size(); i ++)
    {
      if (lid_objects.objects[i].name == "lidbox")
      {
        rail_manipulation_msgs::SegmentedObject obj = lid_objects.objects[i];

        double lower_bound = lid_objects.objects[i].center.z - lid_objects.objects[i].height/2.0 + 0.16;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr lid_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PCLPointCloud2 converter;
        pcl_conversions::toPCL(lid_objects.objects[i].point_cloud, converter);
        pcl::fromPCLPointCloud2(converter, *object_cloud);

        pcl::IndicesPtr filter_indices(new vector<int>);
        filter_indices->resize(object_cloud->points.size());
        for (size_t i = 0; i < object_cloud->points.size(); i++)
        {
          filter_indices->at(i) = i;
        }

        // check bounding areas (bound the inverse of what we want since PCL will return the removed indicies)
        pcl::ConditionOr<pcl::PointXYZRGB>::Ptr bounds(new pcl::ConditionOr<pcl::PointXYZRGB>);
        bounds->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LE, lower_bound))
        );

        // remove past the given bounds
        this->inverseBound(object_cloud, filter_indices, bounds, filter_indices);

        // extract point cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(object_cloud);
        extract.setIndices(filter_indices);
        extract.filter(*lid_cloud);

        obj.name = "lid";
        pcl::toPCLPointCloud2(*lid_cloud, converter);
        pcl_conversions::fromPCL(converter, obj.point_cloud);

        // calculate the new center
        // calculate the bounding box
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*lid_cloud, min_pt, max_pt);
        obj.width = max_pt[0] - min_pt[0];
        obj.depth = max_pt[1] - min_pt[1];
        obj.height = max_pt[2] - min_pt[2];

        // calculate the center
        obj.center.x = (max_pt[0] + min_pt[0]) / 2.0;
        obj.center.y = (max_pt[1] + min_pt[1]) / 2.0;
        obj.center.z = (max_pt[2] + min_pt[2]) / 2.0;

        recognized_objects.objects.push_back(obj);
        state.lid_position = obj.center;

        break;
      }
    }
  }

  for (unsigned int i = 0; i < state.objects.size(); i ++)
  {
    bool was_updated = false;
    for (unsigned int j = 0; j < updated.size(); j ++)
    {
      if (state.objects[i].name == updated[j])
      {
        was_updated = true;
        break;
      }
    }
    if (!was_updated)
    {
      state.objects[i].occluded = true;
    }
    else
    {
      state.objects[i].occluded = false;
    }
  }

  recognized_objects_publisher.publish(recognized_objects);
  segmented_objects_updated = true;
}

bool StateCalculator::updateStateCallback(task_sim_nimbus_bridge::UpdateState::Request &req,
    task_sim_nimbus_bridge::UpdateState::Response &res)
{
  state = req.state;

  state_publisher.publish(state);

  return true;
}

void StateCalculator::inverseBound(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in,
    const pcl::IndicesConstPtr &indices_in,
    const pcl::ConditionBase<pcl::PointXYZRGB>::Ptr &conditions,
    const pcl::IndicesPtr &indices_out) const
{
  // use a temp point cloud to extract the indices
  pcl::PointCloud<pcl::PointXYZRGB> tmp;
  pcl::ConditionalRemoval<pcl::PointXYZRGB> removal(conditions, true);
  removal.setInputCloud(in);
  removal.setIndices(indices_in);
  removal.filter(tmp);
  *indices_out = *removal.getRemovedIndices();
}

//convert from RGB color space to CIELAB color space, taken and adapted from pcl/registration/gicp6d
Eigen::Vector3f RGB2Lab (const Eigen::Vector3f& colorRGB)
{
  // for sRGB   -> CIEXYZ see http://www.easyrgb.com/index.php?X=MATH&H=02#text2
  // for CIEXYZ -> CIELAB see http://www.easyrgb.com/index.php?X=MATH&H=07#text7

  double R, G, B, X, Y, Z;

  R = colorRGB[0];
  G = colorRGB[1];
  B = colorRGB[2];

  // linearize sRGB values
  if (R > 0.04045)
    R = pow ( (R + 0.055) / 1.055, 2.4);
  else
    R = R / 12.92;

  if (G > 0.04045)
    G = pow ( (G + 0.055) / 1.055, 2.4);
  else
    G = G / 12.92;

  if (B > 0.04045)
    B = pow ( (B + 0.055) / 1.055, 2.4);
  else
    B = B / 12.92;

  // postponed:
  //    R *= 100.0;
  //    G *= 100.0;
  //    B *= 100.0;

  // linear sRGB -> CIEXYZ
  X = R * 0.4124 + G * 0.3576 + B * 0.1805;
  Y = R * 0.2126 + G * 0.7152 + B * 0.0722;
  Z = R * 0.0193 + G * 0.1192 + B * 0.9505;

  // *= 100.0 including:
  X /= 0.95047;  //95.047;
  //    Y /= 1;//100.000;
  Z /= 1.08883;  //108.883;

  // CIEXYZ -> CIELAB
  if (X > 0.008856)
    X = pow (X, 1.0 / 3.0);
  else
    X = 7.787 * X + 16.0 / 116.0;

  if (Y > 0.008856)
    Y = pow (Y, 1.0 / 3.0);
  else
    Y = 7.787 * Y + 16.0 / 116.0;

  if (Z > 0.008856)
    Z = pow (Z, 1.0 / 3.0);
  else
    Z = 7.787 * Z + 16.0 / 116.0;

  Eigen::Vector3f colorLab;
  colorLab[0] = static_cast<float> (116.0 * Y - 16.0);
  colorLab[1] = static_cast<float> (500.0 * (X - Y));
  colorLab[2] = static_cast<float> (200.0 * (Y - Z));

  return colorLab;
}
