#ifndef GRAPH_IMAGE_LASER_H
#define GRAPH_IMAGE_LASER_H

#include <string>
#include <map>
#include <vector>
#include <iostream>
#include <ros/ros.h>


#include <laser_slam/parameters.hpp>
#include <laser_slam/incremental_estimator.hpp>
#include <graph_image_laser/laser_image_slam_worker.hpp>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>

#include "graph_image_laser/SaveMap.h"

#include <thread>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../src/visual/loop-closure/loop_closure.h"



//brief特征提取了类
// This functor extracts BRIEF descriptors in the required format
class BriefExtractor: public FeatureExtractor<FBrief::TDescriptor>
{
public:
  virtual void operator()(const cv::Mat &im, const std::vector<cv::Point2f> window_pts,
    std::vector<cv::KeyPoint> &keys, std::vector<BRIEF::bitset> &descriptors) const;
  BriefExtractor(const std::string &pattern_file);

private:
  DVision::BRIEF m_brief;
};

//激光建图的参数
struct graphImageLaserParams
{
  //是否在闭环之后清理局部地图
  bool clear_local_map_after_loop_closure = true;

  //是否发布世界坐标系到里程计坐标系的位姿变换
  bool publish_world_to_odom;
  //世界坐标系
  std::string world_frame;
  //tf发布频率
  double tf_publication_rate_hz;

  double alpha;

  //位姿优化参数
  laser_slam::EstimatorParams online_estimator_params;
}; // struct graphImageLaserParams

class graphImageLaser
{
 public:
  explicit graphImageLaser(ros::NodeHandle& n);
  ~graphImageLaser();

  //建图线程
  void publishMapThread();

  //tf发布线程
  void publishTfThread();

  //分割闭环优化的参数
  void loopProcessThread();

  //词包闭环检测线程
  void dLoopDetector();

 protected:
  //保存地图的服务
  bool saveMapServiceCall(graph_image_laser::SaveMap::Request& request,
                          graph_image_laser::SaveMap::Response& response);

 private:
  //获取ros的参数
  void getParameters();
  ros::NodeHandle& nh_;

  //参数类
  graphImageLaserParams params_;

  tf::TransformBroadcaster tf_broadcaster_;

  ros::ServiceServer save_distant_map_;
  ros::ServiceServer save_map_;

  //增量优化器
  std::shared_ptr<laser_slam::IncrementalEstimator> incremental_estimator_;


//  // SegMatch objects.
//  //分割检测参数
//  segmatch_ros::SegMatchWorkerParams segmatch_worker_params_;
//  //分割检测器
//  segmatch_ros::SegMatchWorker segmatch_worker_;

  //分割检测，闭环优化线程频率
  static constexpr double loopProcessThreadRate_hz = 3.0;

  unsigned int next_track_id_ = 0u;

  //激光的graph_slam
  std::unique_ptr<graph_image_laser::LaserSlamWorker> laser_slam_worker_;
  //激光的graph_slam参数
  graph_image_laser::LaserSlamWorkerParams laser_slam_worker_params_;


//  LoopClosure *loop_closure;
//  std::string pattern_file;
//  std::string voc_file;
//  std::string CAM_NAMES;
//  int id;
//  int minDuration;
//  int minLast;
//  int last_loop_id;
  bool loop_succed;
  std::map<int,laser_slam::Time> loopData;

  mutable std::recursive_mutex loopProcess_mutex_;

  //闭环位姿
  laser_slam::RelativePose loopRelativeclosure;
};

#endif // GRAPH_IMAGE_LASER_H
