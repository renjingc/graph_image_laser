#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#ifdef linux
#include <unistd.h>
#include <dirent.h>
#endif
#ifdef WIN32
#include <direct.h>
#include <io.h>
#endif

using namespace std;

std::vector<Eigen::Matrix4d> load_kitti_poses(const std::string poses_filename,
                                                     bool fail_when_not_found = true)
{
  int id=0;
  std::vector<Eigen::Matrix4d> poses;
  std::ifstream poses_file(poses_filename.c_str());
  if(!poses_file.is_open())
  {
    std::perror((std::string("Unable to open file: ") + poses_filename).c_str());
    if(fail_when_not_found)
    {
      exit(1);
    }
    else
    {
      return poses;
    }
  }

  while(true)
  {
    if(poses_file.eof())
    {
      break;
    }
    else
    {
        if(id%2==1)
        {
            double r1, r2, r3, r4, r5, r6, r7, r8, r9;
            double t1, t2, t3;
            poses_file >> r1 >> r2 >> r3 >> t1 >> r4 >> r5 >> r6 >> t2 >> r7 >> r8 >> r9 >> t3;

            Eigen::Matrix4d pose(Eigen::Matrix4d::Identity());
            pose<<r1, r2, r3, t1,
                  r4, r5, r6, t2,
                  r7, r8, r9, t3,
                  0 ,  0,  0,  1;

            poses.push_back(pose);
        }
        else if(id%2==0)
        {
            double t;
            poses_file >> t;
        }
        id++;
    }
  }

  return poses;
}

vector<string> getFiles(string cate_dir)
{
    vector<string> files;//存放文件名

#ifdef WIN32
    _finddata_t file;
    long lf;
    //输入文件夹路径
    if ((lf=_findfirst(cate_dir.c_str(), &file)) == -1) {
        cout<<cate_dir<<" not found!!!"<<endl;
    } else {
        while(_findnext(lf, &file) == 0) {
            //输出文件名
            //cout<<file.name<<endl;
            if (strcmp(file.name, ".") == 0 || strcmp(file.name, "..") == 0)
                continue;
            files.push_back(file.name);
        }
    }
    _findclose(lf);
#endif

    DIR *dir;
    struct dirent *ptr;
    char base[1000];

    if ((dir=opendir(cate_dir.c_str())) == NULL)
        {
        perror("Open dir error...");
                exit(1);
        }

    while ((ptr=readdir(dir)) != NULL)
    {
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
                continue;
        else if(ptr->d_type == 8)    ///file
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            files.push_back(ptr->d_name);
        else if(ptr->d_type == 10)    ///link file
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            continue;
        else if(ptr->d_type == 4)    ///dir
        {
            files.push_back(ptr->d_name);
            /*
                memset(base,'\0',sizeof(base));
                strcpy(base,basePath);
                strcat(base,"/");
                strcat(base,ptr->d_nSame);
                readFileList(base);
            */
        }
    }
    closedir(dir);

    //排序，按从小到大排序
    sort(files.begin(), files.end());
    return files;
}

tf::TransformBroadcaster *tfBroadcaster2Pointer = NULL;
tf::StampedTransform laserOdometryTrans;
int startFile=0;
int endFile=3000;
int main(int argc,char** argv)
{
    ros::init(argc, argv, "LaserMapper");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("start", startFile, 0);
    private_nh.param("end", endFile, 3000);

    ros::Publisher pub= nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1000);
    //加载路径
    std::vector<Eigen::Matrix4d> poses = load_kitti_poses("/media/ren/99146341-07be-4601-9682-0539688db03f/mypcd3/mapping_pose.txt");

    std::string basePath="/media/ren/99146341-07be-4601-9682-0539688db03f/mypcd3/pcd/";
    vector<string> files=getFiles(basePath);

    tf::TransformBroadcaster tfBroadcaster;
    tfBroadcaster2Pointer = &tfBroadcaster;
    laserOdometryTrans.frame_id_ = "/map";
    laserOdometryTrans.child_frame_id_ = "/base_link";

    ros::Rate r(10);
    if(endFile<=startFile)
    {
      startFile=0;
      endFile=files.size();
    }
    if(startFile<0)
    {
      startFile=0;
    }
    if(endFile>files.size())
      endFile=files.size();

      Eigen::Matrix3d current_angle = poses[0].block<3,3>(0,0);
      Eigen::Vector3d current_r_ = current_angle.eulerAngles(0,1,2);//roll pitch yaw 顺序

      std::cout<<0<<" :"<<poses[0](0,3)<<" "<<poses[0](1,3)<<" "<<current_r_[2]<<std::endl;

      current_angle = poses[5000].block<3,3>(0,0);
      current_r_ = current_angle.eulerAngles(0,1,2);//roll pitch yaw 顺序

      std::cout<<5000<<" :"<<poses[5000](0,3)<<" "<<poses[5000](1,3)<<" "<<current_r_[2]<<std::endl;

      current_angle = poses[10000].block<3,3>(0,0);
      current_r_ = current_angle.eulerAngles(0,1,2);//roll pitch yaw 顺序

      std::cout<<10000<<" :"<<poses[10000](0,3)<<" "<<poses[10000](1,3)<<" "<<current_r_[2]<<std::endl;

      current_angle = poses[13900].block<3,3>(0,0);
      current_r_ = current_angle.eulerAngles(0,1,2);//roll pitch yaw 顺序

      std::cout<<13900<<" :"<<poses[13900](0,3)<<" "<<poses[13900](1,3)<<" "<<current_r_[2]<<std::endl;

    for(int i = startFile; i < endFile && ros::ok(); i++)
    {
      //std::cout<<"pub: "<<i<<" "<<endFile<<std::endl;
      string scan = basePath+files[i];

      pcl::PointCloud<pcl::PointXYZI> cloud;
      pcl::io::loadPCDFile(scan, cloud);

      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(cloud,msg);
      msg.header.frame_id="/base_link";
      msg.header.stamp=ros::Time::now();

      Eigen::Matrix3d current_angle = poses[i].block<3,3>(0,0);
      Eigen::Vector3d current_r_ = current_angle.eulerAngles(0,1,2);//roll pitch yaw 顺序


      tf::Quaternion q;
      q.setRPY(current_r_(0), current_r_(1), current_r_(2));

      laserOdometryTrans.stamp_ = ros::Time::now();
      laserOdometryTrans.setRotation(q);
      laserOdometryTrans.setOrigin(tf::Vector3(poses[i](0,3), poses[i](1,3), poses[i](2,3)));

      tfBroadcaster2Pointer->sendTransform(laserOdometryTrans);
      pub.publish(msg);

      r.sleep();
    }
}
