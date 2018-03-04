#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <thread>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "visual/loop-closure/loop_closure.h"

#include <map>
#include <vector>


using namespace std;
using namespace cv;

string IMAGE_TOPIC="/cam00/image_raw";
string LASER_TOPIC="/velodyne_points";

Mat image;
LoopClosure *loop_closure;
std::string pattern_file="/home/ren/catkin_ws/src/VINS-Mono/support_files/brief_pattern.yml";
std::string voc_file="/home/ren/catkin_ws/src/VINS-Mono/support_files/brief_k10L6.bin";
std::string CAM_NAMES="/home/ren/catkin_ws/src/visual_lidar/config/camera_config.yaml";
bool getImage=false;
int id=0;
int minDuration=150;
int minLast=50;
int last_loop_id=0;

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
/*
 * 接收图像的回调函数
 */
void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat show_img = ptr->image;
    image=show_img;
    getImage=true;
    cv::imshow("show",show_img);
    cv::waitKey(30);
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "dataSave");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    n.param<std::string>("IMAGE_TOPIC", IMAGE_TOPIC, "/cam00/image_raw");
    n.param<std::string>("LASER_TOPIC", LASER_TOPIC, "/velodyne_points");

    //设置原始图像的订阅
    ros::Subscriber sub_raw_image = n.subscribe(IMAGE_TOPIC, 2000, img_callback);

    ros::Rate r(10);
    while(ros::ok())
    {
        if(getImage)
        {
            if(loop_closure == NULL)
            {
                //计时器
                TicToc t_load_voc;
                ROS_DEBUG("loop start loop");
                std::cout << "voc file: " << voc_file << std::endl;
                //加载闭环检测模型，闭环检测类初始化
                loop_closure = new LoopClosure(voc_file.c_str(), image.cols, image.rows,0.8);
                ROS_DEBUG("loop load vocbulary %lf", t_load_voc.toc());
                //初始化闭环模型
                loop_closure->initCameraModel(CAM_NAMES);
            }
            cv::Mat imag1=image;

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

            extractBrief(imag1,pattern_file,keypoints,descriptors);

            //闭环检测时间
            TicToc t_loopdetect;
            //开始检测闭环
            loop_succ = loop_closure->startLoopClosure(keypoints, descriptors, cur_pts, old_pts, old_index);
            ROS_DEBUG("t_loopdetect %f ms", t_loopdetect.toc());
            if(loop_succ && abs(id-old_index)>minDuration && abs(id-last_loop_id)>minLast)
            {
                std::cout<<"loop_succ "<<id<<" "<<old_index<<std::endl;
                last_loop_id=id;
            }
            id++;
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
