#include "graph_image_laser/graph_image_laser.hpp"

#include <stdlib.h>

#include <graph_image_laser/common.hpp>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

using namespace laser_slam;
using namespace graph_image_laser;

cv::Mat image;
LoopClosure *loop_closure;
std::string pattern_file="/home/ren/catkin_ws/src/VINS-Mono/support_files/brief_pattern.yml";
std::string voc_file="/home/ren/catkin_ws/src/VINS-Mono/support_files/brief_k10L6.bin";
std::string CAM_NAMES="/home/ren/catkin_ws/src/visual_lidar/config/camera_config.yaml";
bool getImage=false;
int id=0;
int minDuration=150;
int minLast=50;
int last_loop_id=0;

BriefExtractor::BriefExtractor(const std::string &pattern_file)
{
  // The DVision::BRIEF extractor computes a random pattern by default when
  // the object is created.
  // We load the pattern that we used to build the vocabulary, to make
  // the descriptors compatible with the predefined vocabulary

  // loads the pattern
  cv::FileStorage fs(pattern_file.c_str(), cv::FileStorage::READ);
  if(!fs.isOpened()) throw string("Could not open file ") + pattern_file;

  vector<int> x1, y1, x2, y2;
  fs["x1"] >> x1;
  fs["x2"] >> x2;
  fs["y1"] >> y1;
  fs["y2"] >> y2;

  m_brief.importPairs(x1, y1, x2, y2);
}

void BriefExtractor::operator() (const cv::Mat &im, const std::vector<cv::Point2f> window_pts,
  vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const
{
  //提取fast特征点
  const int fast_th = 20; // corner detector response threshold
  cv::FAST(im, keys, fast_th, true);
//  for(int i = 0; i < (int)window_pts.size(); i++)
//  {
//      cv::KeyPoint key;
//      key.pt = window_pts[i];
//      keys.push_back(key);
//  }
  // 计算每个fast特征点的描述子
  m_brief.compute(im, keys, descriptors);
}


//提取Brief特征
void extractBrief(cv::Mat &image,const std::string &pattern_file,std::vector<cv::KeyPoint> &keys, std::vector<BRIEF::bitset> &descriptors)
{
    BriefExtractor extractor(pattern_file);
    std::vector<cv::Point2f> measurements;
    //提取特征点和描述子
    extractor(image, measurements,keys, descriptors);
//    int start = keypoints.size() - measurements.size();
//    for(int i = 0; i< (int)measurements.size(); i++)
//    {
//        //加入特征和描述子
//        window_keypoints.push_back(keypoints[start + i]);
//        window_descriptors.push_back(descriptors[start + i]);
//    }
}
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

  //创建一个保存地图的服务
  save_map_ = nh_.advertiseService("save_map", &graphImageLaser::saveMapServiceCall, this);

//  pattern_file="/home/ren/catkin_ws/src/VINS-Mono/support_files/brief_pattern.yml";
//  voc_file="/home/ren/catkin_ws/src/VINS-Mono/support_files/brief_k10L6.bin";
//  CAM_NAMES="/home/ren/catkin_ws/src/visual_lidar/config/camera_config.yaml";
//  id=0;
//  minDuration=150;
//  minLast=50;
//  last_loop_id=0;

  loop_succed=false;
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

void graphImageLaser::dLoopDetector()
{
    //发布频率
    ros::Rate thread_rate(10);
    while(ros::ok())
    {
        if(!laser_slam_worker_->cloudImageData.empty())
        {
            std::map<laser_slam::Time, cloudImageFrame>::reverse_iterator it=laser_slam_worker_->cloudImageData.rbegin();
            laser_slam::Time current_time=it->first;
            cv::Mat image1=it->second.image;
            image=image1;
            loopData.emplace(id,current_time);

            if(loop_closure == NULL)
            {
                //计时器
                TicToc t_load_voc;
                ROS_DEBUG("loop start loop");
                std::cout << "voc file: " << voc_file << std::endl;
                //加载闭环检测模型，闭环检测类初始化
                loop_closure = new LoopClosure(voc_file.c_str(), image.cols, image.rows, (float)params_.alpha);
                ROS_DEBUG("loop load vocbulary %lf", t_load_voc.toc());
                //初始化闭环模型
                loop_closure->initCameraModel(CAM_NAMES);
            }

            bool loop_succ = false;
            int old_index = -1;
            //闭环时间
            //当前特征点
            std::vector<cv::Point2f> cur_pts;
            //闭环特征点
            std::vector<cv::Point2f> old_pts;

            //feature descriptor,特征提取
            std::vector<BRIEF::bitset> descriptors;
            //keypoints，特征点
            std::vector<cv::KeyPoint> keypoints;

            extractBrief(image,pattern_file,keypoints,descriptors);
            //闭环检测时间
            TicToc t_loopdetect;
            //开始检测闭环
            loop_succ = loop_closure->startLoopClosure(keypoints, descriptors, cur_pts, old_pts, old_index);
            ROS_DEBUG("t_loopdetect %f ms", t_loopdetect.toc());
            //std::cout<<"loop: "<<t_loopdetect.toc()<<" "<<id<<" "<<old_index<<" "<<last_loop_id<<std::endl;
            if(loop_succ && abs(id-old_index)>minDuration && abs(id-last_loop_id)>minLast)
            {
                std::cout<<"loop_succ "<<id<<" "<<old_index<<std::endl;
                loop_succed=true;
                last_loop_id=id;
                loopRelativeclosure.time_a_ns=loopData[old_index];
                loopRelativeclosure.time_b_ns=loopData[id];
                loopRelativeclosure.track_id_a=loopRelativeclosure.track_id_b=0;
                laser_slam::SO3 loopRelative_r(1.0,0.0,0.0,0.0);
                Eigen::Matrix<double, 3, 1> loopRelative_t;
                loopRelative_t<<0,0,0;
                laser_slam::SE3 loopT(loopRelative_r,loopRelative_t);
                loopRelativeclosure.T_a_b=loopT;
            }
            id++;
        }
        thread_rate.sleep();
    }
}

/*
 * 分割闭环线程
 */
void graphImageLaser::loopProcessThread()
{
    //分割闭环优化线程3Hz
    ros::Rate thread_rate(loopProcessThreadRate_hz);
    while (ros::ok())
    {
        if(loop_succed)
        {
            std::lock_guard<std::recursive_mutex> loopProcessCallback(loopProcess_mutex_);
            //从incremental_estimator_获取当前位姿
            Pose current_pose = incremental_estimator_->getCurrentPose();

            //是闭环
            LOG(INFO) << "Found loop closure! time_a_ns: " << loopRelativeclosure.time_a_ns <<
                " time_b_ns: " << loopRelativeclosure.time_b_ns;

            //用增量优化闭环
            incremental_estimator_->processLoopClosure(loopRelativeclosure,true);

            //是否清空局部地图，一般在闭环之后需要清空
            if (params_.clear_local_map_after_loop_closure)
            {
              laser_slam_worker_->clearLocalMap();
            }

            //更新路径和分割物体
            Trajectory trajectory;
            laser_slam_worker_->getTrajectory(&trajectory);
            laser_slam_worker_->updateFullMap();

            loop_succed=false;

        }
        thread_rate.sleep();
    }
}

/*
 * 保存地图
 */
bool graphImageLaser::saveMapServiceCall(graph_image_laser::SaveMap::Request& request,
                                     graph_image_laser::SaveMap::Response& response)
{
  PointCloud filtered_map,ground_map;
  Trajectory out_trajectory;
  laser_slam_worker_->getFilteredMap(&filtered_map);
  laser_slam_worker_->getGroundMap(&ground_map);
  laser_slam_worker_->getTrajectory(&out_trajectory);
  try
  {
    pcl::io::savePCDFileASCII(request.filename.data+"/map.pcd", filtered_map);
    pcl::io::savePCDFileASCII(request.filename.data+"/ground_map.pcd", ground_map);

    std::ofstream pose_ofs;
    pose_ofs.open(request.filename.data+"/mapping_pose.txt");
    if (!pose_ofs)
    {
      exit(1);
    }
    for(auto& pose:out_trajectory)
    {
        Eigen::Quaterniond q(pose.second.getRotation().w(),pose.second.getRotation().x(),pose.second.getRotation().y(),pose.second.getRotation().z());
        Eigen::Matrix3d r;
        r=q;
        pose_ofs<<pose.first<<"\n";
        pose_ofs << r(0,0)<<" "<<r(0,1)<<" "<<r(0,2)<<" "<<pose.second.getPosition()(0)
                <<" "<<r(1,0)<<" "<<r(1,1)<<" "<<r(1,2)<<" "<<pose.second.getPosition()(1)
                <<" "<<r(2,0)<<" "<<r(2,1)<<" "<<r(2,2)<<" "<<pose.second.getPosition()(2)<<"\n";
    }
    pose_ofs.close();
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
  nh_.getParam(ns + "/alpha",
               params_.alpha);

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
  nh_.getParam("pattern_file",
               pattern_file);
  nh_.getParam("voc_file",
               voc_file);
  nh_.getParam("config_path",
               CAM_NAMES);
}
