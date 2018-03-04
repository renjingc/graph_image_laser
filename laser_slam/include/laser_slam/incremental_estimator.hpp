#ifndef LASER_SLAM_INCREMENTAL_ESTIMATOR_HPP_
#define LASER_SLAM_INCREMENTAL_ESTIMATOR_HPP_

#include <mutex>
#include <unordered_map>
#include <iostream>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "laser_slam/common.hpp"
#include "laser_slam/laser_track.hpp"
#include "laser_slam/parameters.hpp"

namespace laser_slam {

//滑动误差窗口
class IncrementalEstimator
{
 public:
  IncrementalEstimator() {};
  explicit IncrementalEstimator(const EstimatorParams& parameters,
                                unsigned int n_laser_slam_workers = 1u);

  ~IncrementalEstimator() {};

  //处理一个新的闭环.
  void processLoopClosure(const RelativePose& loop_closure,bool empty=false);

  //获取当前位姿
  Pose getCurrentPose(unsigned int laser_track_id = 0u) const
  {
    std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
    return laser_tracks_[laser_track_id]->getCurrentPose();
  };

  //获取track的id
  std::shared_ptr<LaserTrack> getLaserTrack(unsigned int laser_track_id);

  //获取全部的track
  std::vector<std::shared_ptr<LaserTrack> > getAllLaserTracks();

  //建立位姿graph，并估计路径
  gtsam::Values estimate(const gtsam::NonlinearFactorGraph& new_factors,
                         const gtsam::Values& new_values,
                         laser_slam::Time timestamp_ns = 0u);

  //优化并移除一些因子
  gtsam::Values estimateAndRemove(const gtsam::NonlinearFactorGraph& new_factors,
                                  const gtsam::NonlinearFactorGraph& new_associations_factors,
                                  const gtsam::Values& new_values,
                                  const std::vector<unsigned int>& affected_worker_ids,
                                  laser_slam::Time timestamp_ns = 0u);

  //注册先验因子
  gtsam::Values registerPrior(const gtsam::NonlinearFactorGraph& new_factors,
                              const gtsam::Values& new_values,
                              const unsigned int worker_id);

  //获取误差优化的时间
  void getEstimationTimes(std::map<laser_slam::Time, double>* estimation_times) const
  {
    CHECK_NOTNULL(estimation_times);
    *estimation_times = estimation_times_;
  }

 private:
  unsigned int n_laser_slam_workers_;

  //互斥体
  mutable std::recursive_mutex full_class_mutex_;

  //laser_tracks_
  std::vector<std::shared_ptr<LaserTrack> > laser_tracks_;

  //从滑动窗口扫描出来的激光点，固定在世界坐标系下
  std::vector<DataPoints> fixed_scans_;

  //gtsam用于位姿误差优化的islam2
  gtsam::ISAM2 isam2_;

  //icp匹配
  PointMatcher::ICP icp_;

  //闭环噪声模型
  gtsam::noiseModel::Base::shared_ptr loop_closure_noise_model_;
  //第一次关联的噪声模型
  gtsam::noiseModel::Base::shared_ptr first_association_noise_model_;

  //待移除的因子
  std::unordered_map<unsigned int, size_t> factor_indices_to_remove_;
  std::vector<unsigned int> worker_ids_with_removed_prior_;

  std::unordered_map<unsigned int, unsigned int> if_first_then_remove_second_;

  //误差优化时间
  std::map<laser_slam::Time, double> estimation_times_;

  // 优化器参数
  EstimatorParams params_;
}; // IncrementalEstimator

}  // namespace laser_slam

#endif /* LASER_SLAM_INCREMENTAL_ESTIMATOR_HPP_ */
