#ifndef GRAPH_IMAGE_LASER_LASER_IMAGE_SLAM_WORKER_HPP_
#define GRAPH_IMAGE_LASER_LASER_IMAGE_SLAM_WORKER_HPP_

#include <mutex>

#include <laser_slam/common.hpp>
#include <laser_slam/incremental_estimator.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "graph_image_laser/GetLaserTrackSrv.h"
#include "graph_image_laser/common.hpp"


namespace graph_image_laser {

struct cloudImageFrame
{
    cloudImageFrame(){}

    graph_image_laser::PointCloud cloud;
    graph_image_laser::PointCloud ground_cloud;

    //点云位姿
    laser_slam::SE3 T_w_linkpose;

    cv::Mat image;
};

typedef std::map<laser_slam::Time, cloudImageFrame> cloudImageDatabase;

class LaserSlamWorker
{

 public:
  LaserSlamWorker();
  ~LaserSlamWorker();

  void init(ros::NodeHandle& nh, const LaserSlamWorkerParams& params,
            std::shared_ptr<laser_slam::IncrementalEstimator> incremental_estimator,
            unsigned int worker_id = 0u);

  //使用滑动窗口注册局部点云
  void scanCallback(const sensor_msgs::PointCloud2& cloud_msg_in);

  void img_callback(const sensor_msgs::ImageConstPtr &img_msg);
  void ground_scanCallback(const sensor_msgs::PointCloud2& cloud_msg_in);

  //发布机器人轨迹
  void publishTrajectory(const laser_slam::Trajectory& trajectory,
                         const ros::Publisher& publisher) const;

  //发布地图
  void publishMap();

  //只根据轨迹发布估计的归家和里程计
  void publishTrajectories();

  //获取局部滤波后的地图
  void getLocalMapFiltered(graph_image_laser::PointCloud* local_map_filtered);

  //获取滤波后的地图
  void getFilteredMap(graph_image_laser::PointCloud* filtered_map);

  //获取地面地图
  void getGroundMap(graph_image_laser::PointCloud* ground_map);

  //获取优化后的点云向量
  //获取包含自之后记录的优化点云的向量
  //最后一次调用此方法。 此呼叫清除点云队列。
  std::vector<graph_image_laser::PointCloud> getQueuedPoints();

  //清空局部地图
  void clearLocalMap();

  //清空全部地图
  void clearFullMap();

  tf::StampedTransform getWorldToOdom();

  //获取路径
  void getTrajectory(laser_slam::Trajectory* out_trajectory) const;

  //获取里程计路径
  void getOdometryTrajectory(laser_slam::Trajectory* out_trajectory) const;

  //更新局部地图
  void updateLocalMap(const laser_slam::SE3& last_pose_before_update,
                      const laser_slam::Time last_pose_before_update_timestamp_ns);

  //更新全部地图
  void updateFullMap();

  //锁住激光
  void setLockScanCallback(bool new_state);

  //显示Timings
  void displayTimings() const;

  //保存Timings
  void saveTimings() const;

  //每一帧的点云
  cloudImageDatabase cloudImageData;

 private:

  laser_slam::Pose tfTransformToPose(const tf::StampedTransform& tf_transform);

  laser_slam::SE3 geometryMsgTransformToSE3(const geometry_msgs::Transform& transform);
  geometry_msgs::Transform SE3ToGeometryMsgTransform(const laser_slam::SE3& transform);

  // 标准化时间，使轨迹从0开始。
  laser_slam::Time rosTimeToCurveTime(const laser_slam::Time& timestamp_ns);

  laser_slam::Time curveTimeToRosTime(const laser_slam::Time& timestamp_ns) const;

  // 使用ros :: Time（0）表示“使用最新的可用转换”。 可能在重新定位器中
  bool getTransform(const std::string& first_frame,
                    const std::string& second_frame,
                    tf::StampedTransform* transform_ptr,
                    ros::Time transform_time = ros::Time(0));

  bool getLaserTracksServiceCall(graph_image_laser::GetLaserTrackSrv::Request& request,
                                 graph_image_laser::GetLaserTrackSrv::Response& response);

 private:
  LaserSlamWorkerParams params_;

  unsigned int worker_id_;

  // 使laser_track互斥体安全（当添加循环闭合时）。
  std::shared_ptr<laser_slam::LaserTrack> laser_track_;

  // 各种锁
  mutable std::recursive_mutex world_to_odom_mutex_;
  mutable std::recursive_mutex local_map_filtered_mutex_;
  mutable std::recursive_mutex local_map_mutex_;
  mutable std::recursive_mutex full_map_mutex_;
  mutable std::recursive_mutex full_ground_map_mutex_;

  mutable std::recursive_mutex scan_callback_mutex_;
  bool lock_scan_callback_ = false;

  //激光订阅
  ros::Subscriber scan_sub_;
  //地面激光订阅
  ros::Subscriber ground_scan_sub_;
  //图像订阅
  ros::Subscriber sub_raw_image;

  //轨迹和局部和全局地图发布
  ros::Publisher trajectory_pub_;
  ros::Publisher local_map_pub_;
  ros::Publisher full_map_pub_;
  ros::Publisher full_ground_map_pub_;

  // 服务
  ros::ServiceServer get_laser_track_srv_;

  //tf接收器
  tf::TransformListener tf_listener_;

  // 优化器的指针
  std::shared_ptr<laser_slam::IncrementalEstimator> incremental_estimator_;

  //局部地图，局部地图队列
  // 包含由滑动窗口估计的地图。
  // 从local_map_切换到local_map_queue_
  graph_image_laser::PointCloud local_map_;
  std::vector<graph_image_laser::PointCloud> local_map_queue_;

  //局部滤波后的地图
  graph_image_laser::PointCloud local_map_filtered_;

  //全局地图
  graph_image_laser::PointCloud full_map_;
  //全局滤波后的地图
  graph_image_laser::PointCloud full_map_filtered_;

  graph_image_laser::PointCloud full_ground_map_;
  graph_image_laser::PointCloud full_ground_map_filtered_;

  //包含远离传感器并假定为固定的地图。
  //如果机器人重新访问相同的环境，则remote_map_and local_map_将是一个
  //彼此之间，每个密度相同。
  //远距离地图
  graph_image_laser::PointCloud distant_map_;

  //每个测量时间减去时间戳，使轨迹从0开始。
  laser_slam::Time base_time_ns_ = 0;

  // 指示基准时间是否设置。
  bool base_time_set_ = false;

  //上一时刻位姿
  laser_slam::SE3 last_pose_;
  //是否具有上一时刻位姿
  bool last_pose_set_ = false;

  laser_slam::Pose last_pose_sent_to_laser_track_;

  //栅格滤波器
  pcl::VoxelGrid<graph_image_laser::PclPoint> voxel_filter_;

  //世界坐标系与里程计坐标系的变换
  tf::StampedTransform world_to_odom_;


  cv::Mat image;
  bool getImage;

  graph_image_laser::PointCloud ground_cloud;
  bool getGround_cloud;

  //接收tf的延时时间
  static constexpr double kTimeout_s = 0.2;
  //接收激光的缓冲
  static constexpr unsigned int kScanSubscriberMessageQueueSize = 10u;
  //发布的缓冲
  static constexpr unsigned int kPublisherQueueSize = 50u;
}; // LaserSlamWorker

} // namespace graph_image_laser

#endif /* GRAPH_IMAGE_LASER_LASER_SLAM_WORKER_HPP_ */
