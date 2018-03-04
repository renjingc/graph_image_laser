#include "laser_slam/incremental_estimator.hpp"

#include <algorithm>
#include <utility>

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

using namespace gtsam;

namespace laser_slam {

IncrementalEstimator::IncrementalEstimator(const EstimatorParams& parameters,
                                           unsigned int n_laser_slam_workers) : params_(
                                               parameters),
                                               n_laser_slam_workers_(n_laser_slam_workers)
{
  //创建iSAM2
  ISAM2Params isam2_params;
  isam2_params.setRelinearizeSkip(1);
  isam2_params.setRelinearizeThreshold(0.001);
  isam2_ = ISAM2(isam2_params);

  //创建laser tracks，n_laser_slam_workers_默认为1
  for (size_t i = 0u; i < n_laser_slam_workers_; ++i)
  {
    std::shared_ptr<LaserTrack> laser_track(new LaserTrack(parameters.laser_track_params, i));
    laser_tracks_.push_back(std::move(laser_track));
  }

  //创建闭环噪声模型
  using namespace gtsam::noiseModel;
  if (params_.add_m_estimator_on_loop_closures)
  {
    LOG(INFO) << "Creating loop closure noise model with cauchy.";
    loop_closure_noise_model_  = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Sigmas(params_.loop_closure_noise_model));
  }
  else
  {
    loop_closure_noise_model_ =
        gtsam::noiseModel::Diagonal::Sigmas(params_.loop_closure_noise_model);
  }

  //第一个噪声模型
  Eigen::Matrix<double,6,1> first_association_noise_model;
  first_association_noise_model[0] = 0.05;
  first_association_noise_model[1] = 0.05;
  first_association_noise_model[2] = 0.05;
  first_association_noise_model[3] = 0.015;
  first_association_noise_model[4] = 0.015;
  first_association_noise_model[5] = 0.015;
  first_association_noise_model_ =
      gtsam::noiseModel::Diagonal::Sigmas(first_association_noise_model);

  // 使用icp去计算闭环闭环
  // 加载ICP配置以调整闭环转换.
  // 现在使用与激光里程计匹配时相同的配置.
  std::ifstream ifs_icp_configurations(params_.laser_track_params.icp_configuration_file.c_str());
  if (ifs_icp_configurations.good())
  {
    LOG(INFO) << "Loading ICP configurations from: " <<
        params_.laser_track_params.icp_configuration_file;
    icp_.loadFromYaml(ifs_icp_configurations);
  }
  else
  {
    LOG(WARNING) << "Could not open ICP configuration file. Using default configuration.";
    icp_.setDefault();
  }
}

//处理闭环
void IncrementalEstimator::processLoopClosure(const RelativePose& loop_closure,bool empty)
{
  //锁上锁
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);

  if (loop_closure.track_id_a == loop_closure.track_id_b)
  {
    CHECK_LT(loop_closure.time_a_ns, loop_closure.time_b_ns) << "Loop closure has invalid time.";
  }
  CHECK_GE(loop_closure.time_a_ns, laser_tracks_[loop_closure.track_id_a]->getMinTime()) <<
      "Loop closure has invalid time.";
  CHECK_LE(loop_closure.time_a_ns, laser_tracks_[loop_closure.track_id_a]->getMaxTime()) <<
      "Loop closure has invalid time.";
  CHECK_GE(loop_closure.time_b_ns, laser_tracks_[loop_closure.track_id_b]->getMinTime()) <<
      "Loop closure has invalid time.";
  CHECK_LE(loop_closure.time_b_ns, laser_tracks_[loop_closure.track_id_b]->getMaxTime()) <<
      "Loop closure has invalid time.";

  //获取闭环相对位姿
  RelativePose updated_loop_closure = loop_closure;

  // 用里程计计算两帧之间的变换
  SE3 w_T_a_b = loop_closure.T_a_b;
  SE3 T_w_a = laser_tracks_[loop_closure.track_id_a]->evaluate(loop_closure.time_a_ns);
  SE3 T_w_b = laser_tracks_[loop_closure.track_id_b]->evaluate(loop_closure.time_b_ns);
  //a坐标系下的a到b的变换
  SE3 a_T_a_b = T_w_a.inverse() * w_T_a_b * T_w_b;
//  updated_loop_closure.T_a_b = a_T_a_b;

  if(empty)
  {
      SO3 loopRelative_r(1.0,0.0,0.0,0.0);
      Eigen::Matrix<double, 3, 1> loopRelative_t;
      loopRelative_t<<0,0,0;
      SE3 a_T_a_b1(loopRelative_r,loopRelative_t);
      //闭环相对位姿，即正确的相对位姿，若完全重合则是0,0,0
      updated_loop_closure.T_a_b = loop_closure.T_a_b;
  }
  else
  {
      //闭环相对位姿，即正确的相对位姿，若完全重合则是0,0,0
      updated_loop_closure.T_a_b = a_T_a_b;
  }

  // 使用icp
  if (params_.do_icp_step_on_loop_closures)
  {
    // 用里程计初始化高斯
    PointMatcher::TransformationParameters initial_guess =
        updated_loop_closure.T_a_b.getTransformationMatrix().cast<float>();

    LOG(INFO) << "Creating the submaps for loop closure ICP.";
    Clock clock;
    DataPoints sub_map_a;
    DataPoints sub_map_b;
    //获取a和b的附近地图中的点云
    laser_tracks_[updated_loop_closure.track_id_a]->buildSubMapAroundTime(
        loop_closure.time_a_ns, params_.loop_closures_sub_maps_radius, &sub_map_a);
    laser_tracks_[updated_loop_closure.track_id_b]->buildSubMapAroundTime(
        loop_closure.time_b_ns, params_.loop_closures_sub_maps_radius, &sub_map_b);
    clock.takeTime();
    LOG(INFO) << "Took " << clock.getRealTime() << " ms to create loop closures sub maps.";

    LOG(INFO) << "Creating loop closure ICP.";
    clock.start();
    //用icp再进一步计算这两幅点云的相对位姿
    PointMatcher::TransformationParameters icp_solution = icp_.compute(sub_map_b, sub_map_a,
                                                                       initial_guess);
    clock.takeTime();
    LOG(INFO) << "Took " << clock.getRealTime() <<
        " ms to compute the icp_solution for the loop closure.";

    updated_loop_closure.T_a_b = convertTransformationMatrixToSE3(icp_solution);
    std::cout<<updated_loop_closure.T_a_b.getPosition()(0)<<" "<<updated_loop_closure.T_a_b.getPosition()(1)<<" "<<updated_loop_closure.T_a_b.getPosition()(2)<<std::endl;
  }
  LOG(INFO)<<updated_loop_closure.T_a_b;

  LOG(INFO) << "Creating loop closure factor.";

  NonlinearFactorGraph new_factors, new_associations_factors;
  //获取里程计中a和b的识别坐标系的位姿的自动微分
  Expression<SE3> exp_T_w_b(laser_tracks_[loop_closure.track_id_b]->getValueExpression(
      updated_loop_closure.time_b_ns));
  Expression<SE3> exp_T_w_a(laser_tracks_[loop_closure.track_id_a]->getValueExpression(
      updated_loop_closure.time_a_ns));

  Expression<SE3> exp_T_a_w(kindr::minimal::inverse(exp_T_w_a));

  Expression<SE3> exp_relative(kindr::minimal::compose(exp_T_a_w, exp_T_w_b));
  //相对位姿的自动微分，这里是闭环噪声模型
  ExpressionFactor<SE3> new_factor(loop_closure_noise_model_, updated_loop_closure.T_a_b,
                                   exp_relative);

  //加入这个新的因子
  new_factors.push_back(new_factor);
  //相对位姿的自动微分，这里是第一次关联的噪声模型
  ExpressionFactor<SE3> new_association_factor(first_association_noise_model_,
                                               updated_loop_closure.T_a_b, exp_relative);

  new_associations_factors.push_back(new_association_factor);

  //将这两个id加入路径中
  LOG(INFO) << "Estimating the trajectories.";
  std::vector<unsigned int> affected_worker_ids;
  affected_worker_ids.push_back(loop_closure.track_id_a);
  affected_worker_ids.push_back(loop_closure.track_id_b);
  Values new_values;
  //开始优化并移除
  Values result = estimateAndRemove(new_factors, new_associations_factors,
                                    new_values, affected_worker_ids,
                                    updated_loop_closure.time_b_ns);

  //更新track中的路径
  LOG(INFO) << "Updating the trajectories after LC.";
  for (auto& track: laser_tracks_)
  {
    track->updateFromGTSAMValues(result);
  }
  LOG(INFO) << "Updating the trajectories after LC done.";
}

//误差优化，每一帧之间的优化
Values IncrementalEstimator::estimate(const gtsam::NonlinearFactorGraph& new_factors,
                                      const gtsam::Values& new_values,
                                      laser_slam::Time timestamp_ns)
{
  //优化时，锁住full_class_mutex_
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
  Clock clock;
  // 开始优化
  isam2_.update(new_factors, new_values).print();

  isam2_.update();
  isam2_.update();

  Values result(isam2_.calculateEstimate());

  clock.takeTime();
//  LOG(INFO) << "Took " << clock.getRealTime() << "ms to estimate the trajectory.";
  estimation_times_.emplace(timestamp_ns, clock.getRealTime());
  return result;
}

//误差优化并remove掉一些因子，在法线闭环时，进行的优化
Values IncrementalEstimator::estimateAndRemove(
    const gtsam::NonlinearFactorGraph& new_factors,
    const gtsam::NonlinearFactorGraph& new_associations_factors,
    const gtsam::Values& new_values,
    const std::vector<unsigned int>& affected_worker_ids,
    laser_slam::Time timestamp_ns)
{
  //优化时，锁住full_class_mutex_
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);

  Clock clock;
  CHECK_EQ(affected_worker_ids.size(), 2u);
  //构建新增加的非线性因子
  gtsam::NonlinearFactorGraph new_factors_to_add = new_factors;
  //查找并更新要删除的因子索引
  std::vector<size_t> factor_indices_to_remove;
  //判断两个id不是同一个
  if (affected_worker_ids.at(0u) != affected_worker_ids.at(1u))
  {
    // 如果没有删除，删除具有之前没有删除的最大一个ID.
    const unsigned int max_id = std::max(affected_worker_ids.at(0u),
                                         affected_worker_ids.at(1u));

    const unsigned int min_id = std::min(affected_worker_ids.at(0u),
                                         affected_worker_ids.at(1u));

    unsigned int worker_id_to_remove = max_id;

    if (min_id != 0u)
    {
      if_first_then_remove_second_.emplace(worker_id_to_remove,
                                           min_id);
    }

    if (std::find(worker_ids_with_removed_prior_.begin(),
                  worker_ids_with_removed_prior_.end(), worker_id_to_remove) !=
                      worker_ids_with_removed_prior_.end())
    {
      worker_id_to_remove = std::min(affected_worker_ids.at(0u),
                                     affected_worker_ids.at(1u));

    }

    if (factor_indices_to_remove_.count(worker_id_to_remove) != 1u)
    {
      if (min_id == 0u)
      {
        if (if_first_then_remove_second_.find(max_id) !=
            if_first_then_remove_second_.end())
        {
          worker_id_to_remove = if_first_then_remove_second_.at(max_id);
        }
      }
    }

    CHECK_LT(factor_indices_to_remove_.count(worker_id_to_remove), 2u);
    if (factor_indices_to_remove_.count(worker_id_to_remove) == 1u)
    {
      factor_indices_to_remove.push_back(factor_indices_to_remove_.at(worker_id_to_remove));
      factor_indices_to_remove_.erase(worker_id_to_remove);
      worker_ids_with_removed_prior_.push_back(worker_id_to_remove);

      // 如果我们删除先前使用的正确的噪声模型.
      new_factors_to_add = new_associations_factors;
    }
  }

  isam2_.update(new_factors_to_add, new_values, factor_indices_to_remove).print();

  // 必须更新两次
  isam2_.update();
  isam2_.update();

  Values result(isam2_.calculateEstimate());

  clock.takeTime();
  LOG(INFO) << "Took " << clock.getRealTime() << "ms to estimate the trajectory.";
  estimation_times_.emplace(timestamp_ns, clock.getRealTime());
  return result;
}

//注册先验factor
gtsam::Values IncrementalEstimator::registerPrior(const gtsam::NonlinearFactorGraph& new_factors,
                                                  const gtsam::Values& new_values,
                                                  const unsigned int worker_id)
{
  //优化时，锁住full_class_mutex_
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
  //优化结果
  ISAM2Result update_result = isam2_.update(new_factors, new_values);

  CHECK_EQ(update_result.newFactorsIndices.size(), 1u);
  if (worker_id > 0u)
  {
    factor_indices_to_remove_.insert(
        std::make_pair(worker_id, update_result.newFactorsIndices.at(0u)));
  }

  isam2_.update();
  isam2_.update();
  Values result(isam2_.calculateEstimate());
  return result;
}

//获取laser_track
std::shared_ptr<LaserTrack> IncrementalEstimator::getLaserTrack(unsigned int laser_track_id)
{
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
  CHECK_GE(laser_track_id, 0u);
  CHECK_LT(laser_track_id, laser_tracks_.size());
  return laser_tracks_[laser_track_id];
}

//获取全部的laser_track
std::vector<std::shared_ptr<LaserTrack> > IncrementalEstimator::getAllLaserTracks()
{
  std::lock_guard<std::recursive_mutex> lock(full_class_mutex_);
  return laser_tracks_;
}

} // namespace laser_slam
