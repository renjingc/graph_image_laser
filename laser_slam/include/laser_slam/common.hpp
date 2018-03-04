#ifndef LASER_SLAM_COMMON_HPP_
#define LASER_SLAM_COMMON_HPP_

#include <fstream>

#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <mincurves/DiscreteSE3Curve.hpp>
#include <pointmatcher/PointMatcher.h>

#include <opencv2/opencv.hpp>

namespace laser_slam
{

typedef PointMatcher<float> PointMatcher;
typedef typename PointMatcher::DataPoints DataPoints;

//SE3位姿，变换矩阵
typedef kindr::minimal::QuatTransformationTemplate<double> SE3;
//SO3，旋转矩阵
typedef SE3::Rotation SO3;

//当前时间
typedef curves::Time Time;

//Timer类
class Clock
{
 public:
  Clock() { start(); }

  /// \brief 启动时钟定时器.
  void start()
  {
    gettimeofday(&real_time_start_, NULL);
    cpu_start_ = clock();
  }

  /// \brief 采样时钟定时器.
  void takeTime()
  {
    struct timeval end;
    gettimeofday(&end, NULL);
    cpu_time_ms_ = double(clock() - cpu_start_) / CLOCKS_PER_SEC * kSecondsToMiliseconds;

    long seconds, useconds;

    seconds  = end.tv_sec  - real_time_start_.tv_sec;
    useconds = end.tv_usec - real_time_start_.tv_usec;
    real_time_ms_ = (seconds * kSecondsToMiliseconds +
        useconds * kMicrosecondsToMiliseconds) + 0.5;
  }

  /// \brief 返回经过的物理时间.
  double getRealTime() { return real_time_ms_; }

  /// \brief 返回经过CPU时间.
  double getCPUTime() { return cpu_time_ms_; }

  double takeRealTime() { takeTime(); return getRealTime(); }

 private:
  struct timeval real_time_start_;
  double real_time_ms_, cpu_time_ms_;
  clock_t cpu_start_;

  static constexpr double kSecondsToMiliseconds = 1000.0;
  static constexpr double kMicrosecondsToMiliseconds = 0.001;
};

//向量的乘法
static double multiplyVectorsImplementation(Eigen::Vector3d a,
                                            Eigen::Vector3d b,
                                            gtsam::OptionalJacobian<1,3> Ha,
                                            gtsam::OptionalJacobian<1,3> Hb)
{
  if(Ha)
    *Ha = b.transpose();

  if(Hb)
    *Hb = a.transpose();

  return a.transpose() * b;
}

static gtsam::Expression<double> multiplyVectors(const gtsam::Expression<Eigen::Vector3d>& C1,
                                                 const gtsam::Expression<Eigen::Vector3d>& C2)
{
  return gtsam::Expression<double>(&multiplyVectorsImplementation, C1, C2);
}

/// \brief Key type.
typedef size_t Key;

//绝对位姿和时间戳
/// \brief 姿态类型包括绝对变换和时间戳。
struct Pose
{
  /// \brief 绝对位姿
  SE3 T_w;
  /// \brief 时间戳.
  curves::Time time_ns;
  /// \brief key.
  Key key;
};


//相对位姿，A到B的相对位姿，A的时间戳，B的时间戳，A的关键点，B的关键点
/// \brief 相对长度类型，包括相对变换和间隔时间戳。
struct RelativePose
{
  /// \brief 相对位姿变换.
  SE3 T_a_b;
  /// \brief A的时间戳.
  curves::Time time_a_ns;
  /// \brief B的时间戳.
  curves::Time time_b_ns;
  /// \brief 先前a的key.
  Key key_a;
  /// \brief 后一个b的key.
  Key key_b;
  unsigned int track_id_a;
  unsigned int track_id_b;
};

//激光类型
/// \brief LaserScan类型，包括点云和时间戳。
struct LaserScan
{
  /// \brief 激光点
  DataPoints scan;
  /// \brief 时间戳
  curves::Time time_ns;
  /// \brief key.
  Key key;
};

//激光图像类型
/// \brief LaserScan类型，包括点云和时间戳。
struct LaserScanAndImage
{
  /// \brief 激光点
  DataPoints scan;
  /// \brief 时间戳
  curves::Time time_ns;
  /// \brief key.
  Key key;
  //图像
  cv::Mat image;
};

//协方差
typedef Eigen::MatrixXd Covariance;


//对齐声明
template<template<typename, typename> class Container, typename Type>
using Aligned = Container<Type, Eigen::aligned_allocator<Type> >;

//位姿向量，相对位姿向量
typedef Aligned<std::vector, Pose> PoseVector;
typedef Aligned<std::vector, RelativePose> RelativePoseVector;

//基于时间戳的路径
typedef std::map<Time, SE3> Trajectory;

//矫正变换矩阵
// Correct transformation matrix.
static void correctTransformationMatrix(PointMatcher::TransformationParameters* transformation_matrix)
{
  //检查变换矩阵
  CHECK_NOTNULL(transformation_matrix);

  //刚性变换
  PointMatcher::Transformation* rigid_transformation =
      PointMatcher::get().REG(Transformation).create("RigidTransformation");
  CHECK_NOTNULL(rigid_transformation);

  //是否符合预期的约束，这个变换不是刚性变换
  if (!rigid_transformation->checkParameters(*transformation_matrix))
  {
    LOG(WARNING) << "The transformation matrix does not represent a valid rigid "
        << "transformation. Projecting onto an orthogonal basis.";
    *transformation_matrix = rigid_transformation->correctParameters(*transformation_matrix);
  }
}

typedef std::vector<std::string> StringRow;
typedef std::vector<std::vector<std::string> > StringMatrix;

//将矩阵保存为csv文件
static void writeCSV(const StringMatrix& string_matrix, const std::string& filename)
{
  CHECK_GE(string_matrix.size(), 1) << "Provided matrix of strings had no entries.";
  std::ofstream out_file_stream;
  out_file_stream.open(filename.c_str());

  for (StringMatrix::const_iterator it = string_matrix.begin(); it != string_matrix.end(); ++it)
  {
    CHECK_GE(it->size(), 1) << "String matrix row has no entries.";
    out_file_stream << it->at(0u);
    for (size_t i = 1u; i < it->size(); ++i)
    {
      out_file_stream << "," << it->at(i);
    }
    out_file_stream << std::endl;
  }
  out_file_stream.close();
}

//MatriXd到CSV
static void writeEigenMatrixXdCSV(const Eigen::MatrixXd& matrix, const std::string& filename)
{
  StringMatrix string_matrix;
  string_matrix.reserve(matrix.rows());
  StringRow string_row;
  string_row.reserve(matrix.cols());
  for (size_t i = 0u; i < matrix.rows(); ++i)
  {
    string_row.clear();
    for (size_t j = 0u; j < matrix.cols(); ++j)
    {
      string_row.push_back(std::to_string(matrix(i,j)));
    }
    string_matrix.push_back(string_row);
  }
  writeCSV(string_matrix, filename);
}

//读取CSV到matrix
static StringMatrix loadCSV(std::string file_name)
{
  std::ifstream in_file_stream(file_name.c_str());
  CHECK(in_file_stream.good()) << "error opening input file " << file_name;
  StringMatrix string_matrix;
  std::string line;
  while(std::getline(in_file_stream, line))
  {
    std::istringstream ss(line);
    StringRow str_row;
    std::string field;
    while (getline(ss, field,',')) {
      str_row.push_back(field);
    }
    string_matrix.push_back(str_row);
  }
  in_file_stream.close();
  return string_matrix;
}

// 帮助器功能将CSV文件读入Eigen :: MatrixXd。
static void loadEigenMatrixXdCSV(std::string file_name, Eigen::MatrixXd* matrix)
{
  CHECK_NOTNULL(matrix);

  // 将CSV加载到字符串矩阵
  StringMatrix string_matrix = loadCSV(file_name);
  CHECK_GE(string_matrix.size(), 1) << "CSV " << file_name << "was empty.";

  // 迭代CSV的行，并使用逗号分隔的字段来填充输出。
  const unsigned n_rows = string_matrix.size();
  const unsigned n_cols = string_matrix[0].size();
  matrix->resize(n_rows, n_cols);

  for (size_t i = 0u; i < n_rows; ++i) {
    for (size_t j = 0u; j < n_cols; ++j) {
      (*matrix)(i,j) = atof(string_matrix[i][j].c_str());
    }
  }
  LOG(INFO) << "Loaded " << file_name << " with " << n_rows << " rows and " <<
      n_cols << " cols.";
}

//map转matrix
static void toEigenMatrixXd(std::map<Time, double> map_in, Eigen::MatrixXd* matrix_out)
{
  CHECK_NOTNULL(matrix_out);
  matrix_out->resize(map_in.size(),2);

  unsigned int i = 0u;
  for (const auto& elem: map_in) {
    (*matrix_out)(i,0) = elem.first;
    (*matrix_out)(i,1) = elem.second;
    ++i;
  }
}

//优化结果.
struct OptimizationResult
{
  //优化迭代次数
  size_t num_iterations = 0;
  //L-M中间步骤数
  size_t num_intermediate_steps = 0;
  //变量数
  size_t num_variables = 0;
  //初始因子图误差
  double initial_error = 0;
  //最终因子图误差
  double final_error = 0;
  //优化时间
  long duration_ms = 0;
  //cpu优化时间
  long durationCpu_ms = 0;
};

//矩阵转SE3
static SE3 convertTransformationMatrixToSE3(
    const PointMatcher::TransformationParameters& transformation_matrix)
{
  SO3 rotation = SO3::constructAndRenormalize(
      transformation_matrix.cast<double>().topLeftCorner<3,3>());
  SE3::Position position = transformation_matrix.cast<double>().topRightCorner<3,1>();
  return SE3(rotation, position);
}

//SE3之间的距离
static double distanceBetweenTwoSE3(const SE3& pose1, const SE3& pose2)
{
  return std::sqrt(
      (pose1.getPosition()(0) - pose2.getPosition()(0)) *
      (pose1.getPosition()(0) - pose2.getPosition()(0)) +
      (pose1.getPosition()(1) - pose2.getPosition()(1)) *
      (pose1.getPosition()(1) - pose2.getPosition()(1)) +
      (pose1.getPosition()(2) - pose2.getPosition()(2)) *
      (pose1.getPosition()(2) - pose2.getPosition()(2)));
}

//获取均值和方差
static void getMeanAndSigma(std::vector<double> values, double* mean_out,
                            double* sigma_out)
{
  CHECK_NOTNULL(mean_out);
  CHECK_NOTNULL(sigma_out);
  //LOG(INFO) << "values";
  double sum = 0.0;
  const double n = double(values.size());
  for (const auto& value: values) {
    //LOG(INFO) << "value " << value;
    sum += value;
  }
  const double mean = sum / n;

  double sum_of_squared_diff = 0.0;
  for (const auto& value: values) {
    sum_of_squared_diff += (mean - value) * (mean - value);
  }
  *mean_out = mean;
  *sigma_out = std::sqrt(sum_of_squared_diff / n);
}

} // namespace laser_slam

#endif /* LASER_SLAM_COMMON_HPP_ */
