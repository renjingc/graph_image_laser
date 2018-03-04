#include "laser_slam/laser_track.hpp"

#include <gtsam/nonlinear/Marginals.h>

using namespace gtsam;
using namespace curves;

namespace laser_slam {

LaserTrack::LaserTrack(const LaserTrackParams& parameters,
                       unsigned int laser_track_id) : params_(parameters),
                           laser_track_id_(laser_track_id)
{
  //icp设置
  std::ifstream ifs_icp_configurations(params_.icp_configuration_file.c_str());
  if (ifs_icp_configurations.good())
  {
    LOG(INFO) << "Loading ICP configurations from: " << params_.icp_configuration_file;
    icp_.loadFromYaml(ifs_icp_configurations);
  }
  else
  {
    LOG(WARNING) << "Could not open ICP configuration file. Using default configuration.";
    icp_.setDefault();
  }

  //icp输入滤波设置.
  std::ifstream ifs_input_filters(params_.icp_input_filters_file.c_str());
  if (ifs_input_filters.good()) {
    LOG(INFO) << "Loading ICP input filters from: " << params_.icp_input_filters_file;
    input_filters_ = PointMatcher::DataPointsFilters(ifs_input_filters);
  } else {
    LOG(FATAL) << "Could not open ICP input filters configuration file.";
  }

  //创建刚性变换
  rigid_transformation_ = PointMatcher::get().REG(Transformation).create("RigidTransformation");
  CHECK_NOTNULL(rigid_transformation_);

  //创建噪声模型
  using namespace gtsam::noiseModel;
  if (params_.add_m_estimator_on_odom)
  {
    //使用柯西分布
    LOG(INFO) << "Creating odometry noise model with cauchy.";
    odometry_noise_model_  = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Sigmas(params_.odometry_noise_model));
  }
  else
  {
    odometry_noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(params_.odometry_noise_model);
  }

  if (params_.add_m_estimator_on_icp)
  {
    LOG(INFO) << "Creating ICP noise model with cauchy.";
    icp_noise_model_  = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Sigmas(params_.icp_noise_model));
  }
  else
  {
    icp_noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(params_.icp_noise_model);
  }

  Eigen::Matrix<double,6,1> noise;
  noise(0) = 0.0000001;
  noise(1) = 0.0000001;
  noise(2) = 0.0000001;
  noise(3) = 0.0000001;
  noise(4) = 0.0000001;
  noise(5) = 0.0000001;

  //先验分布，对角线矩阵6*6，全都是0.0000001
  prior_noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(noise);
}

//处理位姿，将位姿加入pose_measurements_
void LaserTrack::processPose(const Pose& pose)
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  if (pose_measurements_.empty() && pose.time_ns != 0)
  {
    LOG(WARNING) << "First pose had timestamp different than 0 (" << pose.time_ns << ".";
  }
  pose_measurements_.push_back(pose);
}

//处理激光，
void LaserTrack::processLaserScan(const LaserScan& in_scan)
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  LaserScan scan = in_scan;

  //输入滤波
  Clock clock;
  input_filters_.apply(scan.scan);
  clock.takeTime();
  LOG(INFO) << "Took " << clock.getRealTime() << " ms to filter the input scan.";

  // 计算相对姿势测量，扩展轨迹和
  // 计算ICP转换
  if (trajectory_.isEmpty())
  {
    //第一帧位姿
    scan.key = extendTrajectory(scan.time_ns, getPoseMeasurement(scan.time_ns));
  }
  else
  {
    //上一时刻位姿
    SE3 last_pose_measurement = getPoseMeasurement(trajectory_.getMaxTime());

    //当前位姿
    SE3 new_pose_measurement = getPoseMeasurement(scan.time_ns);

    //相对位姿
    RelativePose relative_measurement;
    relative_measurement.T_a_b = last_pose_measurement.inverse()*new_pose_measurement;
    relative_measurement.time_a_ns = trajectory_.getMaxTime();
    relative_measurement.key_a = getPoseKey(trajectory_.getMaxTime());
    relative_measurement.time_b_ns = scan.time_ns;

    //加入最新的位姿
    scan.key =  extendTrajectory(scan.time_ns, trajectory_.evaluate(trajectory_.getMaxTime()) *
                                 relative_measurement.T_a_b);

    //计算并保存相对位姿odometry_measurements_
    relative_measurement.key_b = scan.key;
    odometry_measurements_.push_back(relative_measurement);

    //使用icp
    if (params_.use_icp_factors)
    {
      computeICPTransformations();
    }
  }

  //使用位姿的key保存scan
  setPoseKey(scan.time_ns, scan.key);
  laser_scans_.push_back(scan);
}

/*
 * 加入新一帧的位姿和激光点
 */
void LaserTrack::processPoseAndLaserScan(const Pose& pose, const LaserScan& in_scan,
                                         gtsam::NonlinearFactorGraph* newFactors,
                                         gtsam::Values* newValues,
                                         bool* is_prior)
{
  //full_laser_track_mutex_锁上
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);

  //match的时间
  Clock scan_matching_clock;

  if (pose.time_ns != in_scan.time_ns)
  {
    LOG(WARNING) << "The time of the pose to add (" << pose.time_ns << ") does not match the " <<
        "time of the scan to add (" << in_scan.time_ns << ").";
  }
  //清空newFactors，newValues
  if (newFactors != NULL)
  {
    CHECK(newFactors->empty());
  }
  if (newValues != NULL)
  {
    newValues->clear();
  }

  //当前激光
  LaserScan scan = in_scan;

  //激光输入滤波
  Clock clock;
  input_filters_.apply(scan.scan);
  clock.takeTime();
    //  LOG(INFO) << "Took " << clock.getRealTime() << " ms to filter the input scan.";

  //pose加入pose_measurements_
  if (pose_measurements_.empty() && pose.time_ns != 0)
  {
    LOG(WARNING) << "First pose had timestamp different than 0 (" << pose.time_ns << ".";
  }
  //加入里程计位姿测量值
  pose_measurements_.push_back(pose);

  //使用icp计算相对位姿，并增加trajectory
  //如果当前路径为空，则是第一镇
  if (trajectory_.isEmpty())
  {
    //加上当前路径，并设置当前scan的key
    scan.key = extendTrajectory(scan.time_ns, getPoseMeasurement(scan.time_ns));

    //更新pose的key和保存激光laser_scans_
    setPoseKey(scan.time_ns, scan.key);
    //加入激光测量值
    laser_scans_.push_back(scan);

    if (newFactors != NULL)
    {
      Pose prior_pose = pose;
      //对于第一个key加入先验，一般为false
      if (params_.force_priors)
      {
        prior_pose.T_w = SE3(SE3::Rotation(1.0, 0.0, 0.0, 0.0),
                             SE3::Position(0.0,
                                           kDistanceBetweenPriorPoses_m * laser_track_id_, 0.0));
      }

      //factor图加入先验位姿factor
      newFactors->push_back(makeMeasurementFactor(prior_pose, prior_noise_model_));
    }
    //有先验，说明是第一帧
    if (is_prior != NULL)
    {
      *is_prior = true;
    }
  }
  //如果不是第一帧
  else
  {
    //上一时刻位姿
    SE3 last_pose_measurement = getPoseMeasurement(trajectory_.getMaxTime());

    //当前位姿
    SE3 new_pose_measurement = getPoseMeasurement(scan.time_ns);

    //相对位姿
    RelativePose relative_measurement;
    relative_measurement.T_a_b = last_pose_measurement.inverse()*new_pose_measurement;
    relative_measurement.time_a_ns = trajectory_.getMaxTime();
    relative_measurement.key_a = getPoseKey(trajectory_.getMaxTime());
    relative_measurement.time_b_ns = scan.time_ns;

    //增加trajectory
    scan.key =  extendTrajectory(scan.time_ns, trajectory_.evaluate(trajectory_.getMaxTime()) *
                                 relative_measurement.T_a_b);

    //更新pose的key和保存激光
    setPoseKey(scan.time_ns, scan.key);
    laser_scans_.push_back(scan);

    //增加odometry_measurements_的key
    relative_measurement.key_b = scan.key;
    //增加里程计的相对测量位置
    odometry_measurements_.push_back(relative_measurement);

    //icp变换
    // 是否使用icp变换，这里闭环的时候使用
    if (params_.use_icp_factors)
    {
      computeICPTransformations();
    }

    //match时间
    scan_matching_clock.takeTime();
    scan_matching_times_.emplace(scan.time_ns, scan_matching_clock.getRealTime());

    if (newFactors != NULL)
    {
      //更新odometry和icp的factor
      if (params_.use_odom_factors)
      {
        //加入里程计的因子
        newFactors->push_back(makeRelativeMeasurementFactor(relative_measurement,
                                                            odometry_noise_model_));
      }
      if (params_.use_icp_factors)
      {
        //加入icp的因子
        newFactors->push_back(makeRelativeMeasurementFactor(
            icp_transformations_[icp_transformations_.size()-1u], icp_noise_model_));
      }
    }
    //无先验
    if (is_prior != NULL)
    {
      *is_prior = false;
    }
  }
  //增加新的值，scan和位姿
  if (newValues != NULL)
  {
    //加入每一个点激光和位姿
    newValues->insert(scan.key, pose.T_w);
  }
}

//获取上一时刻激光
void LaserTrack::getLastPointCloud(DataPoints* out_point_cloud) const
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK_NOTNULL(out_point_cloud);
}

//获取一段时间内的激光
void LaserTrack::getPointCloudOfTimeInterval(const std::pair<Time, Time>& times_ns,
                                             DataPoints* out_point_cloud) const
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK_NOTNULL(out_point_cloud);
  *out_point_cloud = DataPoints();
}

//获取世界坐标系下局部点云
void LaserTrack::getLocalCloudInWorldFrame(const Time& timestamp_ns,
                                           DataPoints* out_point_cloud) const
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK_NOTNULL(out_point_cloud);

  // 找到本地扫描的迭代器。
  std::vector<LaserScan>::const_iterator it = laser_scans_.end();
  do {
    --it;
  } while (it != laser_scans_.begin() && it->time_ns != timestamp_ns);
  CHECK(it->time_ns == timestamp_ns) << "The requested local scan could not be found:";

  // 从轨迹获得刚性转换，以便在世界范围内转换扫描。
  PointMatcher::TransformationParameters transformation_matrix =
      trajectory_.evaluate(timestamp_ns).getTransformationMatrix().cast<float>();
  correctTransformationMatrix(&transformation_matrix);

  // 在世界范围内转换扫描。
  *out_point_cloud = rigid_transformation_->compute(it->scan,transformation_matrix);
}

//获取全部路径队列
void LaserTrack::getTrajectory(Trajectory* trajectory) const
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK_NOTNULL(trajectory)->clear();

  std::vector<Time> trajectory_times_ns;
  trajectory_.getCurveTimes(&trajectory_times_ns);

  for (auto time_ns: trajectory_times_ns) {
    trajectory->emplace(time_ns, trajectory_.evaluate(time_ns));
  }
}

//获取全部点云队列
const std::vector<LaserScan>& LaserTrack::getLaserScans() const
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  return laser_scans_;
}

//获取协方差
void LaserTrack::getCovariances(std::vector<Covariance>* out_covariances) const
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK_NOTNULL(out_covariances)->clear();
  *out_covariances = covariances_;
}

//获取当前位姿
Pose LaserTrack::getCurrentPose() const
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  Pose current_pose;
  if (!trajectory_.isEmpty())
  {
    current_pose.time_ns = getMaxTime();
    current_pose.T_w = trajectory_.evaluate(current_pose.time_ns);
  }
  return current_pose;
}

//获取上一时刻位姿
Pose LaserTrack::getPreviousPose() const
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  Pose previous_pose;
  if (trajectory_.size() > 1u) {
    std::vector<Time> trajectory_times;
    trajectory_.getCurveTimes(&trajectory_times);
    previous_pose.time_ns = *(++trajectory_times.rbegin());
    previous_pose.T_w = trajectory_.evaluate(previous_pose.time_ns);
  }
  return previous_pose;
}

//获取里程计位姿
void LaserTrack::getOdometryTrajectory(Trajectory* trajectory) const
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK_NOTNULL(trajectory)->clear();
  for (const auto& pose: pose_measurements_) {
    trajectory->emplace(pose.time_ns, pose.T_w);
  }
}

//获取最开始的时间
Time LaserTrack::getMinTime() const
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  return trajectory_.getMinTime();
}

//获取最近的时间
Time LaserTrack::getMaxTime() const
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  return trajectory_.getMaxTime();
}

//获取每一时刻激光的时间
void LaserTrack::getLaserScansTimes(std::vector<curves::Time>* out_times_ns) const
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK_NOTNULL(out_times_ns)->clear();
  for (size_t i = 0u; i < laser_scans_.size(); ++i)
  {
    out_times_ns->push_back(laser_scans_[i].time_ns);
  }
}

//在graph中加入先验factors，未用到
void LaserTrack::appendPriorFactors(const Time& prior_time_ns, NonlinearFactorGraph* graph) const
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK_NOTNULL(graph);

  trajectory_.addPriorFactors(graph, prior_time_ns);
}

//用odom加入相对里程计的odom factor，未用到
void LaserTrack::appendOdometryFactors(const curves::Time& optimization_min_time_ns,
                                       const curves::Time& optimization_max_time_ns,
                                       noiseModel::Base::shared_ptr noise_model,
                                       NonlinearFactorGraph* graph) const
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK_NOTNULL(graph);

  for (const auto& odometry_measurement: odometry_measurements_)
  {
    if (odometry_measurement.time_a_ns >= optimization_min_time_ns &&
        odometry_measurement.time_b_ns <= optimization_max_time_ns)
    {
      graph->push_back(makeRelativeMeasurementFactor(odometry_measurement, noise_model));
    }
  }
}

/*
 * 加入icp facor，未用到
 */
void LaserTrack::appendICPFactors(const curves::Time& optimization_min_time_ns,
                                  const curves::Time& optimization_max_time_ns,
                                  noiseModel::Base::shared_ptr noise_model,
                                  NonlinearFactorGraph* graph) const
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK_NOTNULL(graph);

  for (size_t i = 0u; i < icp_transformations_.size(); ++i)
  {
    // 如果第二个节点落在优化窗口内。
    if (icp_transformations_[i].time_b_ns >= optimization_min_time_ns &&
        icp_transformations_[i].time_b_ns <= optimization_max_time_ns)
    {
      // 如果第一个节点也属于优化窗口。
      if (icp_transformations_[i].time_a_ns >= optimization_min_time_ns &&
          icp_transformations_[i].time_a_ns <= optimization_max_time_ns)
      {
        graph->push_back(makeRelativeMeasurementFactor(icp_transformations_[i], noise_model));
      }
      else
      {
        graph->push_back(makeRelativeMeasurementFactor(icp_transformations_[i],
                                                       noise_model, true));
      }
    }
  }
}

/*
 * 加入闭环factors，未用到
 */
void LaserTrack::appendLoopClosureFactors(const curves::Time& optimization_min_time_ns,
                                          const curves::Time& optimization_max_time_ns,
                                          noiseModel::Base::shared_ptr noise_model,
                                          NonlinearFactorGraph* graph) const
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK_NOTNULL(graph);

  for (size_t i = 0u; i < loop_closures_.size(); ++i)
  {
    // 如果第二个节点落在优化窗口内。
    if (loop_closures_[i].time_b_ns >= optimization_min_time_ns &&
        loop_closures_[i].time_b_ns <= optimization_max_time_ns)
    {

      // 如果第一个节点也属于优化窗口。
      if (loop_closures_[i].time_a_ns >= optimization_min_time_ns &&
          loop_closures_[i].time_a_ns <= optimization_max_time_ns)
      {
        graph->push_back(makeRelativeMeasurementFactor(loop_closures_[i], noise_model));
      }
      else
      {
        graph->push_back(makeRelativeMeasurementFactor(loop_closures_[i],
                                                       noise_model, true));
      }
    }
  }
}

/*
 * 初始化GTSAM
 */
void LaserTrack::initializeGTSAMValues(const KeySet& keys, Values* values) const
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  trajectory_.initializeGTSAMValues(keys, values);
}

/*
 * 更新GTSAM
 */
void LaserTrack::updateFromGTSAMValues(const Values& values)
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  trajectory_.updateFromGTSAMValues(values);
}

/*
 * 更新GTSAM协方差，对于每一个key，更新它的协方差
 */
void LaserTrack::updateCovariancesFromGTSAMValues(const gtsam::NonlinearFactorGraph& factor_graph,
                                                  const gtsam::Values& values)
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  gtsam::KeySet keys = factor_graph.keys();
  gtsam::Marginals marginals(factor_graph, values);
  for (const auto& key: keys)
  {
    covariances_.push_back(marginals.marginalCovariance(key));
  }
}

//相对位姿factor
ExpressionFactor<SE3>
LaserTrack::makeRelativeMeasurementFactor(const RelativePose& relative_pose_measurement,
                                          noiseModel::Base::shared_ptr noise_model,
                                          const bool fix_first_node) const
{
  Expression<SE3> T_w_b(trajectory_.getValueExpression(relative_pose_measurement.time_b_ns));

  // 如果fix_first_node为真，则T_w_a将使用常量SE3表达式。 就是这样
  // 节点将被修复而不是该因素的一部分。
  //如果为true,则SE3表示T_w_a
  if (fix_first_node)
  {
    Expression<SE3> T_w_a(trajectory_.evaluate(relative_pose_measurement.time_a_ns));
    Expression<SE3> T_a_w(kindr::minimal::inverse(T_w_a));
    Expression<SE3> relative(kindr::minimal::compose(T_a_w, T_w_b));
    //ExpressionFactor自动微分
    return ExpressionFactor<SE3>(noise_model,relative_pose_measurement.T_a_b, relative);
  }
  else
  {
    Expression<SE3> T_w_a(trajectory_.getValueExpression(relative_pose_measurement.time_a_ns));
    Expression<SE3> T_a_w(kindr::minimal::inverse(T_w_a));
    Expression<SE3> relative(kindr::minimal::compose(T_a_w, T_w_b));
    //ExpressionFactor自动微分
    return ExpressionFactor<SE3>(noise_model,relative_pose_measurement.T_a_b, relative);
  }
}

//测量factor
gtsam::ExpressionFactor<SE3>
LaserTrack::makeMeasurementFactor(const Pose& pose_measurement,
                                  gtsam::noiseModel::Base::shared_ptr noise_model) const
{
  Expression<SE3> T_w(trajectory_.getValueExpression(pose_measurement.time_ns));
  return ExpressionFactor<SE3>(noise_model,pose_measurement.T_w, T_w);
}

//计算icp变换
void LaserTrack::computeICPTransformations()
{
  if (getNumScans() > 1u)
  {
    Clock clock;
    localScanToSubMap();
    clock.takeTime();
//    LOG(INFO) << "Took " << clock.getRealTime() << " ms to compute the ICP transformations.";
  }
}

//当前激光与局部地图计算变换
void LaserTrack::localScanToSubMap()
{
  //最新一帧的激光点
  LaserScan last_scan = laser_scans_.back();
  RelativePose icp_transformation;
  icp_transformation.time_b_ns = last_scan.time_ns;
  //获取前一帧的激光点
  icp_transformation.time_a_ns = laser_scans_[getNumScans() - 2u].time_ns;

  // 转换最后一个（parameters_.nscan_in_sub_map - 1）扫描
  // 在第二次扫描的帧中。
  Clock clock;
  //获取上一帧的位姿
  const SE3 T_w_to_second_last_scan = trajectory_.evaluate(
      laser_scans_[getNumScans() - 2u].time_ns);
  //获取上一帧的激光点
  DataPoints sub_map = laser_scans_[getNumScans() - 2u].scan;
  PointMatcher::TransformationParameters transformation_matrix;
  //将之前的激光点全部加起来为sub_map
  for (size_t i = 0u; i < std::min(getNumScans() - 2u, size_t(params_.nscan_in_sub_map - 1u)); ++i)
  {
    LaserScan previous_scan = laser_scans_[getNumScans() - 3u - i];
    transformation_matrix = (T_w_to_second_last_scan.inverse() *
        trajectory_.evaluate(previous_scan.time_ns)).getTransformationMatrix().cast<float>();

    correctTransformationMatrix(&transformation_matrix);

    sub_map.concatenate(rigid_transformation_->compute(previous_scan.scan,transformation_matrix));
  }
  clock.takeTime();
//  LOG(INFO) << "Took " << clock.getRealTime() << " ms to build the submap.";

  // 从轨迹中获取初始猜测。即计算a和b根据里程计计算的位姿偏差
  SE3 initial_guess = trajectory_.evaluate(icp_transformation.time_a_ns).inverse() *
      trajectory_.evaluate(icp_transformation.time_b_ns);
  //初始猜测
  transformation_matrix = initial_guess.getTransformationMatrix().cast<float>();

  PointMatcher::TransformationParameters icp_solution = transformation_matrix;
  // icp计算
  try
  {
    icp_solution = icp_.compute(last_scan.scan, sub_map, transformation_matrix);
  }

  catch (PointMatcher::ConvergenceError error)
  {
    //LOG(INFO) << "ICP failed to converge: " << error.what();
  }

  //保存icp结果
  if (params_.save_icp_results)
  {
    last_scan.scan.save("/tmp/last_scan.vtk");
    sub_map.save("/tmp/sub_map.vtk");
    correctTransformationMatrix(&transformation_matrix);
    rigid_transformation_->compute(last_scan.scan,transformation_matrix).save(
        "/tmp/last_scan_alligned_by_initial_guess.vtk");
    correctTransformationMatrix(&icp_solution);
    rigid_transformation_->compute(last_scan.scan,icp_solution).save(
        "/tmp/last_scan_alligned_by_solution.vtk");
  }

  //获取当前帧和之前激光的匹配位姿
  icp_transformation.T_a_b = convertTransformationMatrixToSE3(icp_solution);
  icp_transformation.key_a = getPoseKey(icp_transformation.time_a_ns);
  icp_transformation.key_b = getPoseKey(icp_transformation.time_b_ns);
  //加入icp结果
  icp_transformations_.push_back(icp_transformation);
}

/*
 * 根据时间获取位姿
 */
Pose* LaserTrack::findPose(const Time& timestamp_ns)
{
  CHECK(!pose_measurements_.empty()) << "Cannot register the scan as no pose was registered.";
  CHECK_LE(timestamp_ns, pose_measurements_.back().time_ns) << "The requested time ("
      << timestamp_ns << ") does not exist in the pose measurements. Last pose time is "
      << pose_measurements_.back().time_ns << ".";

  // 在请求的时间找到姿态测量的迭代器。
  PoseVector::iterator it = pose_measurements_.end();
  do {
    --it;
  } while (it != pose_measurements_.begin() && it->time_ns != timestamp_ns);

  CHECK_EQ(it->time_ns, timestamp_ns) 
  << "The requested time does not exist in the pose measurements.";

  return &(*it);
}

/*
 * 根据时间获取位姿
 */
Pose LaserTrack::findPose(const Time& timestamp_ns) const
{
  CHECK(!pose_measurements_.empty()) << "Cannot register the scan as no pose was registered.";
  CHECK_LE(timestamp_ns, pose_measurements_.back().time_ns) << "The requested time ("
      << timestamp_ns << ") does not exist in the pose measurements. Last pose time is "
      << pose_measurements_.back().time_ns << ".";

  PoseVector::const_iterator it = pose_measurements_.end();
  do {
    --it;
  } while (it != pose_measurements_.begin() && it->time_ns != timestamp_ns);

  CHECK_EQ(it->time_ns, timestamp_ns)
  << "The requested time does not exist in the pose measurements.";

  return *it;
}

//寻找时间上最近的位姿
Pose LaserTrack::findNearestPose(const Time& timestamp_ns) const
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK(!pose_measurements_.empty()) << "Cannot find nearest pose as no pose was registered.";
  CHECK_LE(timestamp_ns, pose_measurements_.back().time_ns) << "The requested time ("
      << timestamp_ns << ") is later than the latest pose time. Latest pose time is "
      << pose_measurements_.back().time_ns << ".";

  Pose pose;
  pose.time_ns = timestamp_ns;
  pose.T_w = trajectory_.evaluate(timestamp_ns);
  pose.key = Key();

  return pose;
}

/*
 * 将当前位姿加一个key id，扩展trajectory_
 */
Key LaserTrack::extendTrajectory(const Time& timestamp_ns, const SE3& value)
{
  std::vector<Time> times_ns;
  std::vector<SE3> values;
  std::vector<Key> keys;
  times_ns.push_back(timestamp_ns);
  values.push_back(value);
  trajectory_.extend(times_ns, values, &keys);
  CHECK_EQ(keys.size(), 1u);
  return keys[0];
}

std::vector<LaserScan>::const_iterator LaserTrack::getIteratorToScanAtTime(
    const curves::Time& time_ns) const
{
  bool found = false;
  std::vector<LaserScan>::const_iterator it = laser_scans_.begin();
  while (it != laser_scans_.end() && !found) {
    if (it->time_ns == time_ns) {
      found = true;
    } else {
      ++it;
    }
  }
  if (it == laser_scans_.end()) {
    CHECK(false) << "Could not find the scan.";
  }
  CHECK_EQ(time_ns, it->time_ns);
  return it;
}

void LaserTrack::buildSubMapAroundTime(const curves::Time& time_ns,
                                       const unsigned int sub_maps_radius,
                                       DataPoints* sub_map_out) const
{
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
//  LOG(INFO) << "buildSubMapAroundTime " << time_ns << " for track " <<
//      laser_track_id_;
  CHECK_NOTNULL(sub_map_out);
  const SE3 T_w_a = trajectory_.evaluate(time_ns);

  std::vector<LaserScan>::const_iterator it = getIteratorToScanAtTime(time_ns);
  DataPoints sub_map = it->scan;
  std::vector<LaserScan>::const_iterator it_before = it;
  std::vector<LaserScan>::const_iterator it_after = it;

  PointMatcher::TransformationParameters transformation_matrix;

  // Add the scans with decreasing time stamps.
  bool reached_begin = false;
  if (it_before != laser_scans_.begin()) {
    for (unsigned int i = 0u; i < sub_maps_radius; ++i) {
      if (!reached_begin) {
        --it_before;
        if (it_before == laser_scans_.begin()) {
          reached_begin = true;
        }
        transformation_matrix = (T_w_a.inverse() *
            trajectory_.evaluate(it_before->time_ns)).getTransformationMatrix().cast<float>();
        correctTransformationMatrix(&transformation_matrix);
        sub_map.concatenate(rigid_transformation_->compute(it_before->scan,transformation_matrix));
      }
    }
  }

  // Add the scans with increasing time stamps.
  bool reached_end = false;
  for (unsigned int i = 0u; i < sub_maps_radius; ++i) {
    ++it_after;
    if (it_after != laser_scans_.end() && !reached_end) {
      transformation_matrix = (T_w_a.inverse() *
          trajectory_.evaluate(it_after->time_ns)).getTransformationMatrix().cast<float>();
      correctTransformationMatrix(&transformation_matrix);
      sub_map.concatenate(rigid_transformation_->compute(it_after->scan,transformation_matrix));
    } else {
      reached_end = true;
    }
  }

  *sub_map_out = sub_map;
}

} // namespace laser_slam
