#ifndef LASER_SLAM_PARAMETERS_HPP_
#define LASER_SLAM_PARAMETERS_HPP_

#include <Eigen/Dense>

namespace laser_slam {

//激光路径参数
struct LaserTrackParams
{
  //Eigen数据对其
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //里程计的噪声模型
  Eigen::Matrix<double,6,1> odometry_noise_model;
  //icp的噪声模型
  Eigen::Matrix<double,6,1> icp_noise_model;
  //是否优化里程计误差
  bool add_m_estimator_on_odom;
  //是否优化icp误差
  bool add_m_estimator_on_icp;

  //icp设置文件
  std::string icp_configuration_file;
  //icp输入滤波文件
  std::string icp_input_filters_file;

  //是否使用icp因子
  bool use_icp_factors;
  //是否使用里程计因子
  bool use_odom_factors;
  //submap
  int nscan_in_sub_map;
  //是否保存icp结果
  bool save_icp_results;

  bool force_priors;
}; // struct LaserTrackParams

//误差参数
struct EstimatorParams
{
  //闭环误差模型
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double,6,1> loop_closure_noise_model;
  //是否将闭环加入误差优化
  bool add_m_estimator_on_loop_closures;

  //是否在闭环的时候使用icp匹配
  bool do_icp_step_on_loop_closures;
  //闭环局部地图半径
  int loop_closures_sub_maps_radius;

  //路径
  LaserTrackParams laser_track_params;
}; // struct EstimatorParams

} // namespace laser_slam

#endif // LASER_SLAM_PARAMETERS_HPP_
