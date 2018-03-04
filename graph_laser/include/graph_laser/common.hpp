#ifndef GRAPH_LASER_COMMON_HPP_
#define GRAPH_LASER_COMMON_HPP_

#include <laser_slam/parameters.hpp>
#include <laser_slam/incremental_estimator.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <ros/ros.h>

namespace graph_laser
{

typedef pcl::PointXYZ PclPoint;
typedef pcl::PointCloud<PclPoint> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> PointICloud;
typedef PointICloud::Ptr PointICloudPtr;

/*
 * laser_slam参数
 */
struct LaserSlamWorkerParams
{
  // Map creation & filtering parameters.
  double distance_to_consider_fixed;
  //是否使用距离地图
  bool separate_distant_map;
  //是否使用过滤后的地图
  bool create_filtered_map;
  double minimum_distance_to_add_pose;
  //栅格点降维参数
  double voxel_size_m;
  //每个栅格点中最少的点
  int minimum_point_number_per_voxel;

  //局部地图中是否移除地面点
  bool remove_ground_from_local_map = false;
  //移除地面点的参数
  double ground_distance_to_robot_center_m;

  //是否使用里程计的信息
  bool use_odometry_information = true;

  //三个坐标系
  // Frames.
  std::string odom_frame;
  std::string sensor_frame;
  std::string world_frame;

  // Topics.
  std::string assembled_cloud_sub_topic;
  std::string ground_cloud_sub_topic;
  std::string trajectory_pub_topic;
  std::string odometry_trajectory_pub_topic;
  std::string full_map_pub_topic;
  std::string local_map_pub_topic;
  std::string distant_map_pub_topic;
  std::string get_laser_track_srv_topic;
  std::string full_ground_map_pub_topic;
  std::string image_sub_topic;

  //是否发布全局和局部和距离地图
  // Map publication.
  bool publish_local_map;
  bool publish_full_map;
  bool publish_distant_map;
  bool publish_ground_map;

  //地图发布频率
  double map_publication_rate_hz;
}; // struct LaserSlamWorkerParams

/*
 * 获取laser_slam的参数
 */
static LaserSlamWorkerParams getLaserSlamWorkerParams(const ros::NodeHandle& nh,
                                                      const std::string& prefix)
{
  LaserSlamWorkerParams params;
  const std::string ns = prefix + "/LaserSlamWorker";

  nh.getParam(ns + "/distance_to_consider_fixed", params.distance_to_consider_fixed);
  nh.getParam(ns + "/separate_distant_map", params.separate_distant_map);
  nh.getParam(ns + "/create_filtered_map", params.create_filtered_map);
  nh.getParam(ns + "/minimum_distance_to_add_pose", params.minimum_distance_to_add_pose);
  nh.getParam(ns + "/voxel_size_m", params.voxel_size_m);
  nh.getParam(ns + "/minimum_point_number_per_voxel", params.minimum_point_number_per_voxel);


  nh.getParam(ns + "/remove_ground_from_local_map", params.remove_ground_from_local_map);
  nh.getParam(ns + "/ground_distance_to_robot_center_m", params.ground_distance_to_robot_center_m);

  nh.getParam(ns + "/use_odometry_information", params.use_odometry_information);

  nh.getParam(ns + "/odom_frame", params.odom_frame);
  nh.getParam(ns + "/sensor_frame", params.sensor_frame);
  nh.getParam(ns + "/world_frame", params.world_frame);

  nh.getParam(ns + "/publish_local_map", params.publish_local_map);
  nh.getParam(ns + "/publish_full_map", params.publish_full_map);
  nh.getParam(ns + "/publish_distant_map", params.publish_distant_map);
  nh.getParam(ns + "/publish_ground_map", params.publish_ground_map);
  nh.getParam(ns + "/map_publication_rate_hz", params.map_publication_rate_hz);

  nh.getParam(ns + "/assembled_cloud_sub_topic", params.assembled_cloud_sub_topic);
  nh.getParam(ns + "/image_sub_topic", params.image_sub_topic);
  nh.getParam(ns + "/ground_cloud_sub_topic", params.ground_cloud_sub_topic);
  nh.getParam(ns + "/trajectory_pub_topic", params.trajectory_pub_topic);
  nh.getParam(ns + "/odometry_trajectory_pub_topic", params.odometry_trajectory_pub_topic);
  nh.getParam(ns + "/full_map_pub_topic", params.full_map_pub_topic);
  nh.getParam(ns + "/local_map_pub_topic", params.local_map_pub_topic);
  nh.getParam(ns + "/distant_map_pub_topic", params.distant_map_pub_topic);
  nh.getParam(ns + "/get_laser_track_srv_topic", params.get_laser_track_srv_topic);
  nh.getParam(ns + "/full_ground_map_pub_topic", params.full_ground_map_pub_topic);


  return params;
}

static laser_slam::LaserTrackParams getLaserTrackParams(const ros::NodeHandle& nh,
                                                        const std::string& prefix)
{
  laser_slam::LaserTrackParams params;
  const std::string ns = prefix + "/LaserTrack";

  std::vector<float> odometry_noise_model, icp_noise_model;
  constexpr unsigned int kNoiseModelDimension = 6u;
  nh.getParam(ns + "/odometry_noise_model", odometry_noise_model);
  CHECK_EQ(odometry_noise_model.size(), kNoiseModelDimension);
  for (size_t i = 0u; i < 6u; ++i) {
    params.odometry_noise_model[i] = odometry_noise_model.at(i);
  }
  nh.getParam(ns + "/icp_noise_model", icp_noise_model);
  CHECK_EQ(icp_noise_model.size(), kNoiseModelDimension);
  for (size_t i = 0u; i < 6u; ++i) {
    params.icp_noise_model[i] = icp_noise_model.at(i);
  }
  nh.getParam(ns + "/add_m_estimator_on_odom", params.add_m_estimator_on_odom);
  nh.getParam(ns + "/add_m_estimator_on_icp", params.add_m_estimator_on_icp);

  // TODO move loading of icp_configuration_file and icp_input_filters_file to here.
  nh.getParam(ns + "/use_icp_factors", params.use_icp_factors);
  nh.getParam(ns + "/use_odom_factors", params.use_odom_factors);
  nh.getParam(ns + "/nscan_in_sub_map", params.nscan_in_sub_map);
  nh.getParam(ns + "/save_icp_results", params.save_icp_results);

  nh.getParam(ns + "/force_priors", params.force_priors);
  return params;
}

/*
 * 误差优化的参数
 */
static laser_slam::EstimatorParams getOnlineEstimatorParams(const ros::NodeHandle& nh,
                                                            const std::string& prefix)
{
  laser_slam::EstimatorParams params;
  const std::string ns = prefix + "/OnlineEstimator";

  std::vector<float>  loop_closure_noise_model;
  constexpr unsigned int kNoiseModelDimension = 6u;
  nh.getParam(ns + "/loop_closure_noise_model", loop_closure_noise_model);
  CHECK_EQ(loop_closure_noise_model.size(), kNoiseModelDimension);
  for (size_t i = 0u; i < 6u; ++i) {
    params.loop_closure_noise_model[i] = loop_closure_noise_model.at(i);
  }
  nh.getParam(ns + "/add_m_estimator_on_loop_closures", params.add_m_estimator_on_loop_closures);

  nh.getParam(ns + "/do_icp_step_on_loop_closures", params.do_icp_step_on_loop_closures);
  nh.getParam(ns + "/loop_closures_sub_maps_radius", params.loop_closures_sub_maps_radius);

  params.laser_track_params = getLaserTrackParams(nh, ns);

  return params;
}

static PointCloud lpmToPcl(const laser_slam::PointMatcher::DataPoints& cloud_in)
{
  PointCloud cloud_out;
  cloud_out.width = cloud_in.getNbPoints();
  cloud_out.height = 1;
  for (size_t i = 0u; i < cloud_in.getNbPoints(); ++i) {
    PclPoint point;
    point.x = cloud_in.features(0,i);
    point.y = cloud_in.features(1,i);
    point.z = cloud_in.features(2,i);
    cloud_out.push_back(point);
  }
  return cloud_out;
}

static void convert_to_pcl_point_cloud(const sensor_msgs::PointCloud2& cloud_message,
                                       PointICloud* converted)
{
  pcl::PCLPointCloud2 pcl_point_cloud_2;
  pcl_conversions::toPCL(cloud_message, pcl_point_cloud_2);
  pcl::fromPCLPointCloud2(pcl_point_cloud_2, *converted);
}

static void convert_to_point_cloud_2_msg(const PointICloud& cloud,
                                         const std::string& frame,
                                         sensor_msgs::PointCloud2* converted)
{
  CHECK_NOTNULL(converted);
  // Convert to PCLPointCloud2.
  pcl::PCLPointCloud2 pcl_point_cloud_2;
  pcl::toPCLPointCloud2(cloud, pcl_point_cloud_2);
  // Convert to sensor_msgs::PointCloud2.
  pcl_conversions::fromPCL(pcl_point_cloud_2, *converted);
  // Apply frame to msg.
  converted->header.frame_id = frame;
}

static void convert_to_point_cloud_2_msg(const PointCloud& cloud,
                                         const std::string& frame,
                                         sensor_msgs::PointCloud2* converted)
{
  PointICloud cloud_i;
  pcl::copyPointCloud(cloud, cloud_i);
  convert_to_point_cloud_2_msg(cloud_i, frame, converted);
}

/*
 * 圆柱形滤波，remove_point_inside为true为圆柱形外的点，false为内的点
 */
static void applyCylindricalFilter(const PclPoint& center, double radius_m,
                                   double height_m, bool remove_point_inside,
                                   PointCloud* cloud)
{
  CHECK_NOTNULL(cloud);
  PointCloud filtered_cloud;

  const double radius_squared = pow(radius_m, 2.0);
  const double height_halved_m = height_m / 2.0;

  for (size_t i = 0u; i < cloud->size(); ++i)
  {
    //remove_point_inside为true为圆柱形外的点
    if (remove_point_inside)
    {
      if ((pow(cloud->points[i].x - center.x, 2.0)
          + pow(cloud->points[i].y - center.y, 2.0)) >= radius_squared ||
          abs(cloud->points[i].z - center.z) >= height_halved_m)
      {
        filtered_cloud.points.push_back(cloud->points[i]);
      }
    }
    else
    {
      //false为圆柱形内的点
      if ((pow(cloud->points[i].x - center.x, 2.0)
          + pow(cloud->points[i].y - center.y, 2.0)) <= radius_squared &&
          abs(cloud->points[i].z - center.z) <= height_halved_m)
      {
        filtered_cloud.points.push_back(cloud->points[i]);
      }
    }
  }

  filtered_cloud.width = 1;
  filtered_cloud.height = filtered_cloud.points.size();

  *cloud = filtered_cloud;
}

static pcl::PointXYZ positionFromPose(const Eigen::Affine3f &pose) {
  pcl::PointXYZ position(0.0, 0.0, 0.0);
  return pcl::transformPoint(position, pose);
}

static pcl::PointXYZ positionFromPose(const laser_slam::SE3 &pose) {
    pcl::PointXYZ position;
    position.x=pose.getPosition()(0,0);
    position.y=pose.getPosition()(1,0);
    position.z=pose.getPosition()(2,0);

    return position;
}

} // namespace graph_laser

#endif // GRAPH_LASER_COMMON_HPP_
