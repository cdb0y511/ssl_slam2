// Author of SSL_SLAM2: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Trigger.h>
//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "odomEstimationLocalizationClass.h"

OdomEstimationClass odomEstimation;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;

lidar::Lidar lidar_param;
std::string map_path;
std::string savePose_path;
std::string savePCD_path;
ros::Publisher pubLaserOdometry;
ros::Publisher pubMap;
bool is_odom_inited = false;
double total_time =0;
int total_frame=0;

bool savePCDCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    for(int i = 0; i < total_frame; i++) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
        std::string path = savePCD_path + std::to_string(i) + ".pcd";
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*pointcloud_in, indices);
        //coordinate transform
        for (int i = 0; i < (int) pointcloud_in->points.size(); i++){
            double new_x = pointcloud_in->points[i].z;
            double new_y = -pointcloud_in->points[i].x;
            double new_z = -pointcloud_in->points[i].y;
            pointcloud_in->points[i].x = new_x;
            pointcloud_in->points[i].y = new_y;
            pointcloud_in->points[i].z = new_z;
        }
        pcl::io::savePCDFile<pcl::PointXYZRGB>(path, *pointcloud_in);
        pointCloudBuf.pop();
    }
    //ROS_WARN("write feature map to folder ssl_slam2/map ...");
    res.success = true;
    res.message = "write pcd to folder ssl_slam2/pcd ...";
    return true;
}

void velodynePointHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudSurfBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudEdgeBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}


void odom_estimation(){
    std::ofstream stream(savePose_path, std::ios::out);
    stream << std::setprecision(14);
    stream
            << "# Each pose gives the image_tr__pattern transformation (i.e., pattern to image with right-multiplication). Quaternions are written as used by the Eigen library."
            << std::endl;
    stream << "poses:" << std::endl;
    while(1){
        if(!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()){

            //read data
            mutex_lock.lock();
            if(pointCloudSurfBuf.front()->header.stamp.toSec()<pointCloudEdgeBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period){
                pointCloudSurfBuf.pop();
                ROS_INFO("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;  
            }

            if(pointCloudEdgeBuf.front()->header.stamp.toSec()<pointCloudSurfBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period){
                pointCloudEdgeBuf.pop();
                ROS_INFO("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;  
            }

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
            ros::Time pointcloud_time = (pointCloudEdgeBuf.front())->header.stamp;
            mutex_lock.unlock();
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            if(is_odom_inited){
                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();
                odomEstimation.matchPointsToMap(pointcloud_edge_in, pointcloud_surf_in);
                end = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsed_seconds = end - start;
                total_frame++;
                float time_temp = elapsed_seconds.count() * 1000;
                total_time+=time_temp;
                if(total_frame%100==0)
                    ROS_INFO("current_frame: %d, average odom estimation time %f ms \n \n", total_frame, total_time/total_frame);
            }

            Eigen::Quaterniond q_current(odomEstimation.odom.linear());
            Eigen::Vector3d t_current = odomEstimation.odom.translation();

            // publish odometry
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "map"; 
            laserOdometry.child_frame_id = "base_link"; 
            laserOdometry.header.stamp = pointcloud_time;
            laserOdometry.pose.pose.orientation.x = q_current.x();
            laserOdometry.pose.pose.orientation.y = q_current.y();
            laserOdometry.pose.pose.orientation.z = q_current.z();
            laserOdometry.pose.pose.orientation.w = q_current.w();
            laserOdometry.pose.pose.position.x = t_current.x();
            laserOdometry.pose.pose.position.y = t_current.y();
            laserOdometry.pose.pose.position.z = t_current.z();
            pubLaserOdometry.publish(laserOdometry);
            {
                stream << "  - index: " << total_frame-1 << std::endl;
                stream << "    finalCost: " << -1 << std::endl;
                stream << "    tx: " << t_current.x() << std::endl;
                stream << "    ty: " << t_current.y() << std::endl;
                stream << "    tz: " << t_current.z() << std::endl;
                stream << "    qx: " << q_current.x() << std::endl;
                stream << "    qy: " << q_current.y() << std::endl;
                stream << "    qz: " << q_current.z() << std::endl;
                stream << "    qw: " << q_current.w() << std::endl;
                stream.flush();
            }
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(t_current.x(), t_current.y(), t_current.z()) );
            tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
            
            if(total_frame%100==20){
                sensor_msgs::PointCloud2 pointMapMsg;
                pcl::toROSMsg(*(odomEstimation.laserCloudCornerMap) + *(odomEstimation.laserCloudSurfMap) , pointMapMsg);
                pointMapMsg.header.stamp = pointcloud_time;
                pointMapMsg.header.frame_id = "map";
                pubMap.publish(pointMapMsg);             
            }
 
        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    double map_resolution = 0.05;
    double offset_x = 0.0;
    double offset_y = 0.0;
    double offset_yaw = 0.0;

    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/map_resolution", map_resolution);
    nh.getParam("/map_path", map_path);
    nh.getParam("/offset_x", offset_x);
    nh.getParam("/offset_y", offset_y);
    nh.getParam("/offset_yaw", offset_yaw);
    nh.getParam("/savePose_path", savePose_path);
    nh.getParam("/savePCD_path", savePCD_path);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    odomEstimation.init(lidar_param, map_resolution, map_path);
    odomEstimation.setPose(offset_x,offset_y,0.0,0.0,0.0,offset_yaw);
    is_odom_inited = true;

    ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100, velodyneEdgeHandler);
    ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100, velodyneSurfHandler);
    ros::Subscriber subSurfPointCloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 100, velodynePointHandler);

    pubMap = nh.advertise<sensor_msgs::PointCloud2>("/map", 100);
    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);

    //map saving service
    ros::ServiceServer srv_save = nh.advertiseService("save_pcd", savePCDCallback);

    std::thread odom_estimation_process{odom_estimation};

    ros::spin();

    return 0;
}
