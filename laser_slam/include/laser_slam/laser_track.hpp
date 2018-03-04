#ifndef LASER_SLAM_LASER_TRACK_HPP_
#define LASER_SLAM_LASER_TRACK_HPP_

#include <mutex>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <mincurves/DiscreteSE3Curve.hpp>

#include "laser_slam/common.hpp"
#include "laser_slam/parameters.hpp"

namespace laser_slam {

//使用滑动窗口进行点云采集，问题的设置和优化
/// \brief LaserTrack类接口使用滑动窗口估计器进行点云采集
///问题设置和优化。
class LaserTrack
{

 public:
  LaserTrack() {};

  /// \brief 构造函数。
  explicit LaserTrack(const LaserTrackParams& parameters, unsigned int laser_track_id = 0u);

  ~LaserTrack() {};

  //在世界坐标系中处理新的位姿
  void processPose(const Pose& pose);

  //在激光坐标系下处理新的激光
  void processLaserScan(const LaserScan& scan);

  //处理激光和位姿
  void processPoseAndLaserScan(const Pose& pose, const LaserScan& in_scan,
                               gtsam::NonlinearFactorGraph* newFactors = NULL,
                               gtsam::Values* newValues = NULL,
                               bool* is_prior = NULL);

  //获取上一时刻激光
  void getLastPointCloud(DataPoints* out_point_cloud) const;

  //获取一定时间内的激光
  void getPointCloudOfTimeInterval(const std::pair<Time, Time>& times_ns,
                                   DataPoints* out_point_cloud) const;

  //获取世界坐标下的局部点云
  void getLocalCloudInWorldFrame(const Time& timestamp, DataPoints* out_point_cloud) const;

  //获取全部激光
  const std::vector<LaserScan>& getLaserScans() const;

  //获取路径
  void getTrajectory(Trajectory* trajectory) const;

  //获取里程计路径
  void getOdometryTrajectory(Trajectory* out_trajectory) const;

  //获取协方差
  void getCovariances(std::vector<Covariance>* out_covariances) const;

  //获取当前的位姿
  Pose getCurrentPose() const;

  //获取之前的位姿
  Pose getPreviousPose() const;

  //获取开始的时间
  Time getMinTime() const;

  //获取最后的时间
  Time getMaxTime() const;

  //获取激光时间戳
  void getLaserScansTimes(std::vector<Time>* out_times_ns) const;

  //加入先前的因子在因子图中
  void appendPriorFactors(const curves::Time& prior_time_ns,
                          gtsam::NonlinearFactorGraph* graph) const;

  //加入里程计的因子在因子图中
  void appendOdometryFactors(const curves::Time& optimization_min_time_ns,
                             const curves::Time& optimization_max_time_ns,
                             gtsam::noiseModel::Base::shared_ptr noise_model,
                             gtsam::NonlinearFactorGraph* graph) const;

  //加入icp的因子
  /// \brief Append the ICP factors to the factor graph.
  void appendICPFactors(const curves::Time& optimization_min_time_ns,
                        const curves::Time& optimization_max_time_ns,
                        gtsam::noiseModel::Base::shared_ptr noise_model,
                        gtsam::NonlinearFactorGraph* graph) const;

  //加入闭环因子
  void appendLoopClosureFactors(const curves::Time& optimization_min_time_ns,
                                const curves::Time& optimization_max_time_ns,
                                gtsam::noiseModel::Base::shared_ptr noise_model,
                                gtsam::NonlinearFactorGraph* graph) const;

  //初始化gtsam
  void initializeGTSAMValues(const gtsam::KeySet& keys, gtsam::Values* values) const;

  //从gtsam中更新路径
  void updateFromGTSAMValues(const gtsam::Values& values);

  //从GTSAM值更新协方差矩阵。
  void updateCovariancesFromGTSAMValues(const gtsam::NonlinearFactorGraph& factor_graph,
                                        const gtsam::Values& values);

  //获取激光帧数
  size_t getNumScans() {
    std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
    return laser_scans_.size();
  };

  //打印路径
  void printTrajectory() {
    std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
    trajectory_.print("Laser track trajectory");
  };

  //寻找时间戳最近的位姿
  Pose findNearestPose(const Time& timestamp_ns) const;

  //建立时间戳上的局部地图
  void buildSubMapAroundTime(const curves::Time& time_ns,
                             const unsigned int sub_maps_radius,
                             DataPoints* submap_out) const;

  //获取位姿
  gtsam::Expression<SE3> getValueExpression(const curves::Time& time_ns)
  {
    std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
    return trajectory_.getValueExpression(time_ns);
  };
  //获取位姿
  SE3 evaluate(const curves::Time& time_ns) const
  {
    std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
    return trajectory_.evaluate(time_ns);
  }

  //获取匹配时间
  void getScanMatchingTimes(std::map<laser_slam::Time, double>* scan_matching_times) const
  {
    CHECK_NOTNULL(scan_matching_times);
    *scan_matching_times = scan_matching_times_;
  }

  //保存路径
  void saveTrajectory(const std::string& filename) const
  {
    trajectory_.saveCurveTimesAndValues(filename);
  }

 private:
  //增量SE3
  typedef curves::DiscreteSE3Curve CurveType;

  //构建相对位姿误差因子
  gtsam::ExpressionFactor<SE3>
  makeRelativeMeasurementFactor(const RelativePose& relative_pose_measurement,
                                gtsam::noiseModel::Base::shared_ptr noise_model,
                                const bool fix_first_node = false) const;

  //构建位姿误差因子
  gtsam::ExpressionFactor<SE3>
  makeMeasurementFactor(const Pose& pose_measurement,
                        gtsam::noiseModel::Base::shared_ptr noise_model) const;

  //刚性icp匹配
  // Compute rigid ICP transformations according to the selected strategy.
  void computeICPTransformations();

  //局部icp匹配
  // Compute ICP transformation between the last local scan to a concatenation of the
  // previous scans.
  void localScanToSubMap();

  //根据时间获取位姿
  SE3 getPoseMeasurement(const Time& timestamp_ns) const { return findPose(timestamp_ns).T_w; };

  //设置位姿的key id
  void setPoseKey(const Time& timestamp_ns, const Key& key) { findPose(timestamp_ns)->key = key; };

  //获取位姿的key id
  Key getPoseKey(const Time& timestamp_ns) const { return findPose(timestamp_ns).key; };

  //根据时间寻找time的位姿
  Pose* findPose(const Time& timestamp_ns);

  //根据时间寻找time的位姿
  Pose findPose(const Time& timestamp_ns) const;

  //增加路径
  Key extendTrajectory(const Time& timestamp_ns, const SE3& value);

  std::vector<LaserScan>::const_iterator getIteratorToScanAtTime(
      const curves::Time& time_ns) const;

  //路径id
  unsigned int laser_track_id_;

  //位姿队列
  PoseVector pose_measurements_;

  //里程计相对位姿队列
  RelativePoseVector odometry_measurements_;

  //相对icp变换位姿队列
  RelativePoseVector icp_transformations_;

  //相对闭环位姿
  RelativePoseVector loop_closures_;

  //激光队列数据
  std::vector<LaserScan> laser_scans_;

  //增量路径
  CurveType trajectory_;

  //track互斥锁
  mutable std::recursive_mutex full_laser_track_mutex_;

  //协方差
  std::vector<Covariance> covariances_;

  //icp匹配模块
  PointMatcher::ICP icp_;

  //icp滤波模块
  PointMatcher::DataPointsFilters input_filters_;

  //刚性变换结果
  PointMatcher::Transformation* rigid_transformation_;

  //噪声模型
  gtsam::noiseModel::Base::shared_ptr prior_noise_model_;
  gtsam::noiseModel::Base::shared_ptr odometry_noise_model_;
  gtsam::noiseModel::Base::shared_ptr icp_noise_model_;

  //匹配时间
  std::map<laser_slam::Time, double> scan_matching_times_;

  //路径参数
  LaserTrackParams params_;

  static constexpr double kDistanceBetweenPriorPoses_m = 35.0;
};

}  // namespace laser_slam

#endif /* LASER_SLAM_LASER_TRACK_HPP_ */
