#include "graph_image_laser/laser_image_slam_worker.hpp"

#include "laser_slam/benchmarker.hpp"

//TODO clean
#include <Eigen/Eigenvalues>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <laser_slam/common.hpp>
#include <graph_image_laser/common.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pointmatcher_ros/point_cloud.h>
#include <pointmatcher_ros/transform.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace graph_image_laser {

using namespace laser_slam;

LaserSlamWorker::LaserSlamWorker() { }

LaserSlamWorker::~LaserSlamWorker() { }

/*
 * 后端处理激光初始化
 */
void LaserSlamWorker::init(
    ros::NodeHandle& nh, const LaserSlamWorkerParams& params,
    std::shared_ptr<laser_slam::IncrementalEstimator> incremental_estimator,
    unsigned int worker_id)
{
  params_ = params;
  incremental_estimator_ = incremental_estimator;
  worker_id_ = worker_id;

  //从增量估计器中获取激光跟踪的目标
  laser_track_ = incremental_estimator_->getLaserTrack(worker_id);

  //设置激光订阅
  scan_sub_ = nh.subscribe(params_.assembled_cloud_sub_topic, kScanSubscriberMessageQueueSize,
                           &LaserSlamWorker::scanCallback, this);

  //设置地面激光订阅
  ground_scan_sub_ = nh.subscribe(params_.ground_cloud_sub_topic, kScanSubscriberMessageQueueSize,
                           &LaserSlamWorker::ground_scanCallback, this);

  //设置原始图像的订阅,"/camera/left/image_color"
  sub_raw_image = nh.subscribe(params_.image_sub_topic, 2000, &LaserSlamWorker::img_callback,this);


  // 发布路径
  trajectory_pub_ = nh.advertise<nav_msgs::Path>(params_.trajectory_pub_topic,
                                                 kPublisherQueueSize, true);

  //发布局部地图
  if (params_.publish_local_map)
  {
    local_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(params_.local_map_pub_topic,
                                                            kPublisherQueueSize);
  }

  // 设置服务器端激光Track的回调函数
  get_laser_track_srv_ = nh.advertiseService(
      params_.get_laser_track_srv_topic,
      &LaserSlamWorker::getLaserTracksServiceCall, this);

  //设置栅格滤波器的叶子大小
  voxel_filter_.setLeafSize(params_.voxel_size_m, params_.voxel_size_m,
                            params_.voxel_size_m);
  voxel_filter_.setMinimumPointsNumberPerVoxel(params_.minimum_point_number_per_voxel);

  // 设置初始世界坐标系和里程计坐标系的变换
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matrix;
  matrix.resize(4, 4);
  matrix = Eigen::Matrix<float, 4,4>::Identity();
  world_to_odom_ = PointMatcher_ros::eigenMatrixToStampedTransform<float>(
      matrix, params_.world_frame, params_.odom_frame, ros::Time::now());

  if(params_.publish_full_map)
  {
      full_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(params_.full_map_pub_topic,
                                                             kPublisherQueueSize);
  }
  if(params_.publish_ground_map)
  {
      full_ground_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(params_.full_ground_map_pub_topic,
                                                             kPublisherQueueSize);
  }

  getImage=false;
}
void LaserSlamWorker::img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat show_img = ptr->image;
    //image=show_img;
    cv::resize(show_img,image,cv::Size(640,480));
    getImage=true;
//    cv::imshow("show",image);
//    cv::waitKey(30);
}

void LaserSlamWorker::ground_scanCallback(const sensor_msgs::PointCloud2& cloud_msg_in)
{
    pcl::fromROSMsg(cloud_msg_in,ground_cloud);
    getGround_cloud=true;
}

/*
 * 获取每一帧的激光
 */
void LaserSlamWorker::scanCallback(const sensor_msgs::PointCloud2& cloud_msg_in)
{
  std::lock_guard<std::recursive_mutex> lock_scan_callback(scan_callback_mutex_);
  //自动上锁
  if (!lock_scan_callback_)
  {
    //获取里程计变换，这里从前段里程计中获取
    if (tf_listener_.waitForTransform(params_.odom_frame, params_.sensor_frame,
                                      cloud_msg_in.header.stamp, ros::Duration(kTimeout_s)))
    {
        //odom和传感器坐标系的变换
        tf::StampedTransform tf_transform;
        tf_listener_.lookupTransform(params_.odom_frame, params_.sensor_frame,
                                   cloud_msg_in.header.stamp, tf_transform);

        //是否处理激光点
        bool process_scan = false;
        //当前位姿
        SE3 current_pose;

        //上一时刻位姿
        if (!last_pose_set_)
        {
            process_scan = true;
            last_pose_set_ = true;
            //上一时刻位姿
            last_pose_ = tfTransformToPose(tf_transform).T_w;
        }
        else
        {
            //当前位姿
            current_pose = tfTransformToPose(tf_transform).T_w;
            //当前位姿和上一时刻位姿距离
            float dist_m = distanceBetweenTwoSE3(current_pose, last_pose_);
            //如果大于一定距离，则处理激光点，上一时刻位姿为当前位姿，这里为1m
            if (dist_m > params_.minimum_distance_to_add_pose)
            {
              //处理激光，即为关键帧
              process_scan = true;
              //更新上一时刻位姿
              last_pose_ = current_pose;
            }
        }

        //处理激光
        if (process_scan)
        {
            //将PointCloud2转换到LaserScan
            LaserScan new_scan;
            new_scan.scan = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloud_msg_in);
            new_scan.time_ns = rosTimeToCurveTime(cloud_msg_in.header.stamp.toNSec());

            //处理新的非线性优化因子
            gtsam::NonlinearFactorGraph new_factors;
            gtsam::Values new_values;
            //是否是先验帧，即是否是第一帧，第一帧要做先验，即固定住
            bool is_prior;
            //是否使用里程计信息,一般是使用
            if (params_.use_odometry_information)
            {
                laser_track_->processPoseAndLaserScan(tfTransformToPose(tf_transform), new_scan,
                                                &new_factors, &new_values, &is_prior);
            }
            else
            {
                Pose new_pose;
                Time new_pose_time_ns = tfTransformToPose(tf_transform).time_ns;

                if (laser_track_->getNumScans() > 2u)
                {
                    Pose current_pose = laser_track_->getCurrentPose();

                    if (current_pose.time_ns > new_pose_time_ns - current_pose.time_ns)
                    {
                        Time previous_pose_time = current_pose.time_ns -
                            (new_pose_time_ns - current_pose.time_ns);
                        if (previous_pose_time >= laser_track_->getMinTime() &&
                            previous_pose_time <= laser_track_->getMaxTime())
                        {
                            SE3 previous_pose = laser_track_->evaluate(previous_pose_time);
                            new_pose.T_w = last_pose_sent_to_laser_track_.T_w *
                            previous_pose.inverse()  * current_pose.T_w ;
                            new_pose.T_w = SE3(SO3::fromApproximateRotationMatrix(
                            new_pose.T_w.getRotation().getRotationMatrix()), new_pose.T_w.getPosition());
                        }
                    }
                }

               new_pose.time_ns = new_pose_time_ns;
               laser_track_->processPoseAndLaserScan(new_pose, new_scan,
                                                    &new_factors, &new_values, &is_prior);

               last_pose_sent_to_laser_track_ = new_pose;
            }

            //处理新的因子，优化
            gtsam::Values result;
            //如果是有先验的因子则注册先验因子
            if (is_prior)
            {
                result = incremental_estimator_->registerPrior(new_factors, new_values, worker_id_);
            }
            else
            {
                result = incremental_estimator_->estimate(new_factors, new_values, new_scan.time_ns);
            }

            //更新路径
            laser_track_->updateFromGTSAMValues(result);

            //调整世界坐标系和里程计坐标系
            //获取优化后的当前位姿
            Pose current_pose = laser_track_->getCurrentPose();
            //这个相当于里程计的位姿
            SE3 T_odom_sensor = tfTransformToPose(tf_transform).T_w;
            //这个相当于当前位姿
            SE3 T_w_sensor = current_pose.T_w;
            //这个相当于世界坐标系到里程计坐标系位姿的变换
            SE3 T_w_odom = T_w_sensor * T_odom_sensor.inverse();

            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matrix;
            matrix.resize(4, 4);
            matrix = T_w_odom.getTransformationMatrix().cast<float>();

            {
              std::lock_guard<std::recursive_mutex> lock_world_to_odom(world_to_odom_mutex_);
              //将Eigen::Matrix变为tf::StampedTransform
              world_to_odom_ = PointMatcher_ros::eigenMatrixToStampedTransform<float>(
                  matrix, params_.world_frame, params_.odom_frame, cloud_msg_in.header.stamp);
            }

            //发布里程计
            publishTrajectories();

            //获取上一时刻在世界坐标系下的激光点云
            DataPoints new_fixed_cloud;
            laser_track_->getLocalCloudInWorldFrame(laser_track_->getMaxTime(), &new_fixed_cloud);

            // Transform the cloud in sensor frame
            //TODO(Renaud) move to a transformPointCloud() fct.
            //      laser_slam::PointMatcher::TransformationParameters transformation_matrix =
            //          T_w_sensor.inverse().getTransformationMatrix().cast<float>();
            //
            //      laser_slam::correctTransformationMatrix(&transformation_matrix);
            //
            //      laser_slam::PointMatcher::Transformation* rigid_transformation =
            //          laser_slam::PointMatcher::get().REG(Transformation).create("RigidTransformation");
            //      CHECK_NOTNULL(rigid_transformation);
            //
            //      laser_slam::PointMatcher::DataPoints fixed_cloud_in_sensor_frame =
            //          rigid_transformation->compute(new_fixed_cloud,transformation_matrix);
            //
            //
            //      new_fixed_cloud_pub_.publish(
            //          PointMatcher_ros::pointMatcherCloudToRosMsg<float>(fixed_cloud_in_sensor_frame,
            //                                                             params_.sensor_frame,
            //                                                             cloud_msg_in.header.stamp));

            PointCloud new_fixed_cloud_pcl = lpmToPcl(new_fixed_cloud);

            //是否移除地面点
            if (params_.remove_ground_from_local_map)
            {
              //机器人当前高度
              const double robot_height_m = current_pose.T_w.getPosition()(2);
              //移除地面点后的点
              PointCloud new_fixed_cloud_no_ground;
              for (size_t i = 0u; i < new_fixed_cloud_pcl.size(); ++i)
              {
                //当点z大于一定值，则加入
                if (new_fixed_cloud_pcl.points[i].z > robot_height_m -
                    params_.ground_distance_to_robot_center_m)
                {
                  new_fixed_cloud_no_ground.push_back(new_fixed_cloud_pcl.points[i]);
                }
              }
              new_fixed_cloud_no_ground.width = 1;
              new_fixed_cloud_no_ground.height = new_fixed_cloud_no_ground.points.size();
              new_fixed_cloud_pcl = new_fixed_cloud_no_ground;
            }

            //将新的点云加入到local_map_中
            if (params_.create_filtered_map)
            {
              if (new_fixed_cloud_pcl.size() > 0u)
              {
                //自动上锁
                std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
                std::lock_guard<std::recursive_mutex> lock1(full_map_mutex_);
                std::lock_guard<std::recursive_mutex> lock2(full_ground_map_mutex_);

                Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> transform_matrix;
                transform_matrix.resize(4, 4);
                transform_matrix = T_w_sensor.getTransformationMatrix().cast<float>();
                graph_image_laser::PointCloud transform_ground_cloud;

                if(ground_cloud.size()>0)
                    pcl::transformPointCloud(ground_cloud, transform_ground_cloud, transform_matrix);

                //增加局部地图
                if (local_map_.size() > 0u)
                {
                  local_map_ += new_fixed_cloud_pcl;
                  full_map_ += new_fixed_cloud_pcl;
                }
                else
                {
                  local_map_ = new_fixed_cloud_pcl;
                  full_map_ += new_fixed_cloud_pcl;

                }
                if(ground_cloud.size()>0)
                  full_ground_map_ +=transform_ground_cloud;
                //增加队列
                local_map_queue_.push_back(new_fixed_cloud_pcl);

                if(getImage)
                {
                    cloudImageFrame cloud_image_frame;
                    cloud_image_frame.cloud=new_fixed_cloud_pcl;
                    cloud_image_frame.ground_cloud=transform_ground_cloud;
                    cloud_image_frame.image=image;
                    cloud_image_frame.T_w_linkpose=T_w_sensor;
                    cloudImageData.emplace(new_scan.time_ns,cloud_image_frame);
                }
              }
            }
        }
    }
    else
    {
//      ROS_WARN_STREAM("[SegMapper] Timeout while waiting between " + params_.odom_frame  +
//                      " and " + params_.sensor_frame  + ".");
    }
  }
}

/*
 * 锁住激光回调函数
 */
void LaserSlamWorker::setLockScanCallback(bool new_state)
{
  std::lock_guard<std::recursive_mutex> lock(scan_callback_mutex_);
  lock_scan_callback_ = new_state;
}

/*
 * 获取LaserTrack服务
 */
bool LaserSlamWorker::getLaserTracksServiceCall(
    graph_image_laser::GetLaserTrackSrv::Request& request,
    graph_image_laser::GetLaserTrackSrv::Response& response)
{
  std::vector<std::shared_ptr<LaserTrack> > laser_tracks =
      incremental_estimator_->getAllLaserTracks();
  Trajectory trajectory;
  ros::Time scan_stamp;
  tf::StampedTransform tf_transform;
  geometry_msgs::TransformStamped ros_transform;
  for (const auto& track: laser_tracks)
  {
    track->getTrajectory(&trajectory);
    for (const auto& scan: track->getLaserScans())
    {
      // 获取scan时间
      scan_stamp.fromNSec(curveTimeToRosTime(scan.time_ns));

      response.laser_scans.push_back(
          PointMatcher_ros::pointMatcherCloudToRosMsg<float>(scan.scan,
                                                             params_.sensor_frame,
                                                             scan_stamp));
      tf_transform = PointMatcher_ros::eigenMatrixToStampedTransform<float>(
          trajectory.at(scan.time_ns).getTransformationMatrix().cast<float>(),
          params_.world_frame,
          params_.sensor_frame,
          scan_stamp);
      tf::transformStampedTFToMsg(tf_transform, ros_transform);
      response.transforms.push_back(ros_transform);
    }
  }
  return true;
}

/*
 * 发布路径
 */
void LaserSlamWorker::publishTrajectory(const Trajectory& trajectory,
                                        const ros::Publisher& publisher) const
{
  nav_msgs::Path traj_msg;
  traj_msg.header.frame_id = params_.world_frame;
  Time traj_time = curveTimeToRosTime(trajectory.rbegin()->first);
  traj_msg.header.stamp.fromNSec(traj_time);

  //将每一时刻位姿发布出来
  for (const auto& timePose : trajectory)
  {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = traj_msg.header;
    pose_msg.header.stamp.fromNSec(curveTimeToRosTime(timePose.first));

    pose_msg.pose.position.x = timePose.second.getPosition().x();
    pose_msg.pose.position.y = timePose.second.getPosition().y();
    pose_msg.pose.position.z = timePose.second.getPosition().z();
    pose_msg.pose.orientation.w = timePose.second.getRotation().w();
    pose_msg.pose.orientation.x = timePose.second.getRotation().x();
    pose_msg.pose.orientation.y = timePose.second.getRotation().y();
    pose_msg.pose.orientation.z = timePose.second.getRotation().z();
    traj_msg.poses.push_back(pose_msg);
  }
  publisher.publish(traj_msg);
}

/*
 * 发布地图
 */
void LaserSlamWorker::publishMap()
{
  // TODO 线程安全
  if (local_map_.size() > 0)
  {
    PointCloud filtered_map;
    //滤波地图，获取
    getFilteredMap(&filtered_map);

    //maximumNumberPointsFilter(&filtered_map);
    if(params_.publish_full_map)
    {
        sensor_msgs::PointCloud2 msg;
        convert_to_point_cloud_2_msg(full_map_filtered_,params_.world_frame,&msg);
        full_map_pub_.publish(msg);
    }

    if(params_.publish_ground_map)
    {
        sensor_msgs::PointCloud2 msg;
        convert_to_point_cloud_2_msg(full_ground_map_filtered_,params_.world_frame,&msg);
        full_ground_map_pub_.publish(msg);
    }

    //是否发布局部地图
    if (params_.publish_local_map)
    {
      sensor_msgs::PointCloud2 msg;
      {
        std::lock_guard<std::recursive_mutex> lock(local_map_filtered_mutex_);
        convert_to_point_cloud_2_msg(local_map_filtered_, params_.world_frame, &msg);
      }
      //发布局部地图，发布滤波后的80m内的地图
      local_map_pub_.publish(msg);
    }
  }
}

void LaserSlamWorker::clearFullMap()
{
    std::lock_guard<std::recursive_mutex> lock(full_map_mutex_);
    std::lock_guard<std::recursive_mutex> ground_lock(full_ground_map_mutex_);
    full_map_.clear();
    full_ground_map_.clear();
}

void LaserSlamWorker::updateFullMap()
{
    {
        std::lock_guard<std::recursive_mutex> lock(full_map_mutex_);
        std::lock_guard<std::recursive_mutex> ground_lock(full_ground_map_mutex_);
        full_map_.clear();
        full_ground_map_.clear();

        Trajectory trajectory;
        laser_track_->getTrajectory(&trajectory);
        //遍历每一个有效的分割点云
        for (auto& cloudImageFrame: cloudImageData)
        {
          //获取路径中当前时间的位姿
          SE3 new_pose = trajectory.at(cloudImageFrame.first);

          //获取变换
          SE3 transformation = new_pose * cloudImageFrame.second.T_w_linkpose.inverse();
          // 转换点云
    //      transformPointCloud(transformation, &cloudImageFrame.second.cloud);

          //获取对应的点云
          graph_image_laser::PointCloud* point_cloud=&cloudImageFrame.second.cloud;
          CHECK_NOTNULL(point_cloud);
          //变换矩阵
          const Eigen::Matrix4f transform_matrix = transformation.getTransformationMatrix().cast<float>();
          //转换点云
          pcl::transformPointCloud(*point_cloud, *point_cloud, transform_matrix);


          //获取对应的点云
          graph_image_laser::PointCloud* ground_point_cloud=&cloudImageFrame.second.ground_cloud;
          CHECK_NOTNULL(ground_point_cloud);
          if(ground_point_cloud->size()>0)
          {
              //变换矩阵
              const Eigen::Matrix4f transform_matrix = transformation.getTransformationMatrix().cast<float>();
              //转换点云
              pcl::transformPointCloud(*ground_point_cloud, *ground_point_cloud, transform_matrix);
              cloudImageFrame.second.ground_cloud=*ground_point_cloud;
              full_ground_map_ +=*ground_point_cloud;
          }

          // 更新该点云的位姿
          cloudImageFrame.second.T_w_linkpose = new_pose;
          cloudImageFrame.second.cloud=*point_cloud;
          full_map_+=*point_cloud;

        }
    }
}

/*
 * 发布路径的接口
 */
void LaserSlamWorker::publishTrajectories()
{
  Trajectory trajectory;
  laser_track_->getTrajectory(&trajectory);
  publishTrajectory(trajectory, trajectory_pub_);
}

// StampedTransform转成Pose
Pose LaserSlamWorker::tfTransformToPose(const tf::StampedTransform& tf_transform)
{
  Pose pose;
  SE3::Position pos(tf_transform.getOrigin().getX(), tf_transform.getOrigin().getY(),
                    tf_transform.getOrigin().getZ());
  SE3::Rotation::Implementation rot(tf_transform.getRotation().getW(),
                                    tf_transform.getRotation().getX(),
                                    tf_transform.getRotation().getY(),
                                    tf_transform.getRotation().getZ());
  pose.T_w = SE3(pos, rot);
  pose.time_ns = rosTimeToCurveTime(tf_transform.stamp_.toNSec());

  return pose;
}

Time LaserSlamWorker::rosTimeToCurveTime(const Time& timestamp_ns)
{
  if (!base_time_set_)
  {
    base_time_ns_ = timestamp_ns;
    base_time_set_ = true;
  }
  return timestamp_ns - base_time_ns_;
}

Time LaserSlamWorker::curveTimeToRosTime(const Time& timestamp_ns) const
{
  CHECK(base_time_set_);
  return timestamp_ns + base_time_ns_;
}

/*
 * 获取队列中全部点云
 */
std::vector<graph_image_laser::PointCloud> LaserSlamWorker::getQueuedPoints()
{
  std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
  std::vector<graph_image_laser::PointCloud> new_points;
  new_points.swap(local_map_queue_);
  return new_points;
}

// 滤波地图
//local_map_为没滤波的圆柱形内的点
//local_map_filtered_为滤波后的圆柱形内的点
//distant_map_为滤波后圆柱形外面的点
//filtered_map为原始local_map_滤波后的点
void LaserSlamWorker::getFilteredMap(PointCloud* filtered_map)
{
  BENCHMARK_BLOCK(LS_getFilteredMap);
  //获取当前位姿
  laser_slam::Pose current_pose = laser_track_->getCurrentPose();

  PclPoint current_position;
  current_position.x = current_pose.T_w.getPosition()[0];
  current_position.y = current_pose.T_w.getPosition()[1];
  current_position.z = current_pose.T_w.getPosition()[2];

  //local_map_为没滤波的圆柱形内的点
  //local_map_filtered_为滤波后的圆柱形内的点
  //distant_map_为滤波后圆柱形外面的点
  //filtered_map为原始local_map_滤波后的点
  // 将圆柱形过滤器应用在本地图上并获取副本。
  PointCloud local_map;
  {
    std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
    local_map = local_map_;
    BENCHMARK_BLOCK(LS_cylindricalFilter);
    //圆柱形滤波，取当取位姿80米圆柱形内点
    applyCylindricalFilter(current_position, params_.distance_to_consider_fixed,
                           40, false, &local_map_);
  }

  PointCloud full_map;
  {
    std::lock_guard<std::recursive_mutex> lock(full_map_mutex_);
    full_map = full_map_;
  }
  PointCloud full_ground_map;
  {
    std::lock_guard<std::recursive_mutex> lock(full_ground_map_mutex_);
    full_ground_map = full_ground_map_;
  }

  // 使用栅格滤波
  laser_slam::Clock clock;

  PointCloudPtr local_map_ptr(new PointCloud());
  pcl::copyPointCloud<PclPoint, PclPoint>(local_map, *local_map_ptr);

  PointCloudPtr full_map_ptr(new PointCloud());
  pcl::copyPointCloud<PclPoint, PclPoint>(full_map, *full_map_ptr);


  PointCloud local_map_filtered;
  PointCloud full_map_filtered;

  voxel_filter_.setInputCloud(local_map_ptr);
  voxel_filter_.filter(local_map_filtered);

  voxel_filter_.setInputCloud(full_map_ptr);
  voxel_filter_.filter(full_map_filtered);

  clock.takeTime();

  if (params_.separate_distant_map)
  {
    // 如果分离映射被启用，则local_map_中的每个点之间的距离将会
    // 与当前的机器人位置进行比较。 远离机器人的点
    // 转移到remote_map_。 这对发布有帮助（在remote_map_中的点
    // 需要仅被过滤一次）以及需要完成的任何其他处理
    // 当地图远离机器人时，可以假定为静态（直到环闭合）。

    // TODO（renaud）有没有办法分离云，而不必在传感器中进行转换
    // 通过设置位置来计算距离？
    // 在传感器框架中转换local_map_。
    clock.start();

    // 保存之前滤波前的点
    PointCloud new_distant_map = local_map_filtered;

    //圆柱形滤波，取当取位姿80米圆柱形内点
    applyCylindricalFilter(current_position, params_.distance_to_consider_fixed,
                           40, false, &local_map_filtered);

    //圆柱形滤波，取当取位姿80米圆柱形外点
    applyCylindricalFilter(current_position, params_.distance_to_consider_fixed,
                           40, true, &new_distant_map);

    {
      std::lock_guard<std::recursive_mutex> lock(local_map_filtered_mutex_);
      local_map_filtered_ = local_map_filtered;
    }

    // 将new_distant_map添加到distant_map_。
    if (distant_map_.size() > 0u)
    {
      distant_map_ += new_distant_map;
    }
    else
    {
      distant_map_ = new_distant_map;
    }

//    *filtered_map = local_map_filtered;
//    *filtered_map += distant_map_;

    full_map_filtered_=full_map_filtered;
    full_ground_map_filtered_=full_ground_map;

    *filtered_map = full_map_filtered_;

    clock.takeTime();
    // LOG(INFO) << "new_local_map.size() " << local_map.size();
    // LOG(INFO) << "new_distant_map.size() " << new_distant_map.size();
    // LOG(INFO) << "distant_map_.size() " << distant_map_.size();
    // LOG(INFO) << "Separating done! Took " << clock.getRealTime() << " ms.";
  }
  else
  {
    *filtered_map = local_map;
  }
}

void LaserSlamWorker::getGroundMap(PointCloud* ground_map)
{
    std::lock_guard<std::recursive_mutex> lock(full_ground_map_mutex_);
    *ground_map=full_ground_map_;
}

/*
 * 获取局部滤波后的点local_map_filtered
 */
void LaserSlamWorker::getLocalMapFiltered(PointCloud* local_map_filtered)
{
  CHECK_NOTNULL(local_map_filtered);
  std::lock_guard<std::recursive_mutex> lock(local_map_filtered_mutex_);
  *local_map_filtered = local_map_filtered_;
}

/*
 * 清除局部点local_map_和local_map_filtered_
 */
void LaserSlamWorker::clearLocalMap()
{
  {
    std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
    local_map_.clear();
  }

  {
    std::lock_guard<std::recursive_mutex> lock(local_map_filtered_mutex_);
    local_map_filtered_.clear();
  }
}

/*
 * 获取世界坐标系到里程计坐标系的变换
 */
tf::StampedTransform LaserSlamWorker::getWorldToOdom()
{
  std::lock_guard<std::recursive_mutex> lock_world_to_odom(world_to_odom_mutex_);
  tf::StampedTransform world_to_odom = world_to_odom_;
  return world_to_odom;
}

/*
 * 获取优化后的路径
 */
void LaserSlamWorker::getTrajectory(Trajectory* out_trajectory) const
{
  laser_track_->getTrajectory(out_trajectory);
}

/*
 * 获取单纯里程计的路径
 */
void LaserSlamWorker::getOdometryTrajectory(Trajectory* out_trajectory) const
{
  laser_track_->getOdometryTrajectory(out_trajectory);
}

/*
 * 更新局部地图
 */
void LaserSlamWorker::updateLocalMap(const SE3& last_pose_before_update,
                                     const laser_slam::Time last_pose_before_update_timestamp_ns)
{
  //获取路径
  Trajectory new_trajectory;
  laser_track_->getTrajectory(&new_trajectory);

  SE3 new_last_pose = new_trajectory.at(last_pose_before_update_timestamp_ns);

  //获取新的位姿和优化前的位姿的变换，根据这个变换变换local_map_和local_map_filtered_
  const Eigen::Matrix4f transform_matrix = (new_last_pose * last_pose_before_update.inverse()).
      getTransformationMatrix().cast<float>();
  {
        std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
        pcl::transformPointCloud(local_map_, local_map_, transform_matrix);
  }
  {
        std::lock_guard<std::recursive_mutex> lock(local_map_filtered_mutex_);
        pcl::transformPointCloud(local_map_filtered_, local_map_filtered_, transform_matrix);
  }
}

void LaserSlamWorker::displayTimings() const
{
  /*std::vector<double> scan_matching_times;
  laser_track_->getScanMatchingTimes(&scan_matching_times);

  double mean, sigma;
  getMeanAndSigma(scan_matching_times, &mean, &sigma);
  LOG(INFO) << "Scan matching times for worker id " << worker_id_ <<
      ": " << mean << " +/- " << sigma;

  std::vector<double> estimation_times;
  incremental_estimator_->getEstimationTimes(&estimation_times);
  getMeanAndSigma(estimation_times, &mean, &sigma);
  LOG(INFO) << "Estimation times for worker id " << worker_id_ <<
      ": " << mean << " +/- " << sigma;

  incremental_estimator_->getEstimationAndRemoveTimes(&estimation_times);
  getMeanAndSigma(estimation_times, &mean, &sigma);
  LOG(INFO) << "Estimation and remove times for worker id " << worker_id_ <<
      ": " << mean << " +/- " << sigma;

   */
}

/*
 * 保存scan_matching_times的运行时间和里程计位姿
 */
void LaserSlamWorker::saveTimings() const
{
  std::map<Time, double> scan_matching_times;
  Eigen::MatrixXd matrix;
  laser_track_->getScanMatchingTimes(&scan_matching_times);
  toEigenMatrixXd(scan_matching_times, &matrix);
  writeEigenMatrixXdCSV(matrix, "/tmp/timing_icp_" + std::to_string(worker_id_) + ".csv");

  std::map<Time, double> estimation_times;
  incremental_estimator_->getEstimationTimes(&estimation_times);
  toEigenMatrixXd(estimation_times, &matrix);
  writeEigenMatrixXdCSV(matrix, "/tmp/timing_estimation.csv");

  Trajectory traj;
  laser_track_->getTrajectory(&traj);
  matrix.resize(traj.size(), 4);
  unsigned int i = 0u;
  for (const auto& pose: traj)
  {
    matrix(i,0) = pose.first;
    matrix(i,1) = pose.second.getPosition()(0);
    matrix(i,2) = pose.second.getPosition()(1);
    matrix(i,3) = pose.second.getPosition()(2);
    ++i;
  }
  writeEigenMatrixXdCSV(matrix, "/tmp/trajectory_" + std::to_string(worker_id_) + ".csv");


  /*double mean, sigma;
  getMeanAndSigma(scan_matching_times, &mean, &sigma);
  LOG(INFO) << "Scan matching times for worker id " << worker_id_ <<
      ": " << mean << " +/- " << sigma;

  std::vector<double> estimation_times;
  incremental_estimator_->getEstimationTimes(&estimation_times);
  getMeanAndSigma(estimation_times, &mean, &sigma);
  LOG(INFO) << "Estimation times for worker id " << worker_id_ <<
      ": " << mean << " +/- " << sigma;

  incremental_estimator_->getEstimationAndRemoveTimes(&estimation_times);
  getMeanAndSigma(estimation_times, &mean, &sigma);
  LOG(INFO) << "Estimation and remove times for worker id " << worker_id_ <<
      ": " << mean << " +/- " << sigma;*/
}


} // namespace graph_image_laser
