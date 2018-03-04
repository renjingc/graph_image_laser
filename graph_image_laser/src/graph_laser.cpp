#include "graph_image_laser/graph_laser.hpp"

#include <stdlib.h>

#include <graph_image_laser/common.hpp>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

using namespace laser_slam;
using namespace graph_image_laser;
using namespace segmatch;
using namespace segmatch_ros;

graphImageLaser::graphImageLaser(ros::NodeHandle& n) : nh_(n)
{
  //获取ros的参数
  getParameters();

  //创建增量优化器
  std::shared_ptr<IncrementalEstimator> incremental_estimator(
      new IncrementalEstimator(params_.online_estimator_params));

  //std::move 将一个左值强制转化为右值引用
  incremental_estimator_ = std::move(incremental_estimator);

  //设置laserSlam
  std::unique_ptr<LaserSlamWorker> laser_slam_worker(new LaserSlamWorker());

  //初始化laser_slam
  laser_slam_worker->init(nh_, laser_slam_worker_params_, incremental_estimator_);
  laser_slam_worker_ = std::move(laser_slam_worker);

  //初始化SegMatchWorker
  if (segmatch_worker_params_.localize || segmatch_worker_params_.close_loops)
  {
    segmatch_worker_.init(n, segmatch_worker_params_);
  }

  //创建一个保存地图的服务
  save_map_ = nh_.advertiseService("save_map", &graphImageLaser::saveMapServiceCall, this);
}

graphImageLaser::~graphImageLaser() {}

/*
 * 发布建图线程
 */
void graphImageLaser::publishMapThread()
{
  //是否发布地图
  if (laser_slam_worker_params_.create_filtered_map)
  {
    ros::Rate thread_rate(laser_slam_worker_params_.map_publication_rate_hz);
    while (ros::ok())
    {
      //发布地图
      laser_slam_worker_->publishMap();
      thread_rate.sleep();
    }
  }
}

/*
 * 发布tf线程
 */
void graphImageLaser::publishTfThread()
{
  //是否发布世界坐标系到里程计坐标系的变换
  if (params_.publish_world_to_odom)
  {
    //发布频率
    ros::Rate thread_rate(params_.tf_publication_rate_hz);
    while (ros::ok())
    {
      //从laser_slam_worker获取世界坐标系到里程计的变换
      //发布坐标
      tf::StampedTransform world_to_odom = laser_slam_worker_->getWorldToOdom();
      world_to_odom.stamp_ = ros::Time::now();
      tf_broadcaster_.sendTransform(world_to_odom);
      thread_rate.sleep();
    }
  }
}

/*
 * 分割闭环线程
 */
void graphImageLaser::segMatchThread()
{
  //定位还是闭环模式
  if (segmatch_worker_params_.localize || segmatch_worker_params_.close_loops)
  {
    //分割闭环优化线程3Hz
    ros::Rate thread_rate(kSegMatchThreadRate_hz);
    while (ros::ok())
    {
      //增加函数去检测地图是否更新
      //获取局部地图
      segmatch::PointCloud local_map_filtered;
      laser_slam_worker_->getLocalMapFiltered(&local_map_filtered);

      //局部地图大小>0
      if (local_map_filtered.points.size() > 0)
      {
        //获取source点云
        segmatch::PointICloud source_cloud;
        pcl::copyPointCloud(local_map_filtered, source_cloud);

        //从incremental_estimator_获取当前位姿
        Pose current_pose = incremental_estimator_->getCurrentPose();

        //当定位的时候
        //将局部地图和当前位姿送入分割检测器中
        if (segmatch_worker_params_.localize)
        {
          //处理目标点云和当前位姿
          segmatch_worker_.processSourceCloud(source_cloud, current_pose);
        }
        //当闭环时
        else
        {
          //闭环位姿
          RelativePose loop_closure;
          //闭环
          if (segmatch_worker_.processSourceCloud(source_cloud, current_pose, 0u,
                                                  &loop_closure))
          {
            //是闭环
            LOG(INFO) << "Found loop closure! time_a_ns: " << loop_closure.time_a_ns <<
                " time_b_ns: " << loop_closure.time_b_ns;

            //用增量优化闭环
            incremental_estimator_->processLoopClosure(loop_closure);

            //是否清空局部地图，一般在闭环之后需要清空
            if (params_.clear_local_map_after_loop_closure)
            {
              laser_slam_worker_->clearLocalMap();
            }

            //更新路径和分割物体
            Trajectory trajectory;
            laser_slam_worker_->getTrajectory(&trajectory);
            laser_slam_worker_->updateFullMap();

            //更新分割匹配线程中的路径
            segmatch_worker_.update(trajectory);
            LOG(INFO) << "SegMatchThread updating segmap done";
          }
        }
      }

      thread_rate.sleep();
    }
  }
}

/*
 * 保存地图
 */
bool graphImageLaser::saveMapServiceCall(graph_image_laser::SaveMap::Request& request,
                                     graph_image_laser::SaveMap::Response& response)
{
  PointCloud filtered_map;
  laser_slam_worker_->getFilteredMap(&filtered_map);
  try
  {
    pcl::io::savePCDFileASCII(request.filename.data, filtered_map);
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR_STREAM("Unable to save: " << e.what());
    return false;
  }
  return true;
}

void graphImageLaser::getParameters()
{
  // LaserMapper参数
  const std::string ns = "/graphImageLaser";

  nh_.getParam(ns + "/publish_world_to_odom",
               params_.publish_world_to_odom);
  nh_.getParam(ns + "/world_frame",
               params_.world_frame);
  nh_.getParam(ns + "/tf_publication_rate_hz",
               params_.tf_publication_rate_hz);

  //是否在闭环之后清理局部地图
  nh_.getParam(ns + "/clear_local_map_after_loop_closure",
               params_.clear_local_map_after_loop_closure);

  // laser_slam worker参数
  laser_slam_worker_params_ = graph_image_laser::getLaserSlamWorkerParams(nh_, ns);
  laser_slam_worker_params_.world_frame = params_.world_frame;

  //在线位姿优化的参数
  params_.online_estimator_params = graph_image_laser::getOnlineEstimatorParams(nh_, ns);

  //icp参数
  nh_.getParam("icp_configuration_file",
               params_.online_estimator_params.laser_track_params.icp_configuration_file);
  nh_.getParam("icp_input_filters_file",
               params_.online_estimator_params.laser_track_params.icp_input_filters_file);

  //SegMatch参数
  segmatch_worker_params_ = segmatch_ros::getSegMatchWorkerParams(nh_, ns);
  segmatch_worker_params_.world_frame = params_.world_frame;
}
