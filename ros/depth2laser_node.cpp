#include "depth2laser.h"

//ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>

//opencv
#include <opencv2/opencv.hpp>
//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
//C++
#include <stdio.h>
#include <math.h>
#include <string>

using namespace std;
//Topics and messages
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ros::Subscriber info_sub;
ros::Subscriber frame_sub;
ros::Publisher cloud_pub;
ros::Publisher laser_pub;
sensor_msgs::CameraInfo camerainfo;
sensor_msgs::PointCloud cloud;
sensor_msgs::LaserScan scan;
float inverse_angle_increment;


//EIGEN
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Eigen::Matrix3f K;
Eigen::Matrix3f inv_K;
Eigen::Isometry3f camera_transform;
Eigen::Isometry3f laser_transform;
Eigen::Isometry3f camera2laser_transform;


//Other
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
struct Configuration{
    //TF STUFF
    string laser_frame_id;
    string base_frame_id;

    // laser_parameters
    double angle_min;
    double angle_max;
    int num_ranges;
    double range_min;
    double range_max;
    double laser_plane_thickness;

    //TOPICS
    string pointcloud_topic;
    string camera_image_topic;
    string camera_info_topic;
    string laser_topic;

    //OPTIONS
    int publish_pointcloud;
};
Configuration c;

void clearScan(){
    scan.ranges.resize(c.num_ranges);
    scan.angle_min=c.angle_min;
    scan.angle_max=c.angle_max;
    scan.range_min=c.range_min;
    scan.range_max=c.range_max;
    scan.angle_increment=(c.angle_max-c.angle_min)/c.num_ranges;
    scan.scan_time=0;//*
    scan.time_increment=0;//*

    inverse_angle_increment=1./scan.angle_increment;
    for (size_t i=0; i<scan.ranges.size(); i++)
    {
        scan.ranges[i]=c.range_max+0.1;
    }
}

Eigen::Isometry3f tfTransform2eigen(const tf::Transform& p)
{
    Eigen::Isometry3f iso;
    iso.translation().x()=p.getOrigin().x();//原始数据
    iso.translation().y()=p.getOrigin().y();
    iso.translation().z()=p.getOrigin().z();
    Eigen::Quaternionf q;
    tf::Quaternion tq = p.getRotation();//旋转
    q.x()= tq.x();
    q.y()= tq.y();
    q.z()= tq.z();
    q.w()= tq.w();
    iso.linear()=q.toRotationMatrix();
    return iso;
}

LaserConfig Laser_config;
Depth2Laser d2l;
static int skip_frames=2;
static int skip_count=0;
void frameCallback(const sensor_msgs::Image::ConstPtr& frame)
{
    //...

    if (!d2l.gotInfo()) return;


    if (skip_count<skip_frames) { skip_count++; return; }
    skip_count=0;

    ros::Time current_time = ros::Time::now();

    cloud.points.clear();
    clearScan();

    scan.header.frame_id=c.laser_frame_id;

    cv_bridge::CvImageConstPtr image =  cv_bridge::toCvShare(frame);

    d2l.setImage(image->image);

    d2l.compute();

    scan.header.stamp=current_time;
    scan.ranges = d2l.ranges();
    
    if(c.publish_pointcloud)
    {
      cloud.header.frame_id=frame->header.frame_id;
      cloud.header.stamp= frame->header.stamp;
      PointCloud3f cloud3f = d2l.cloud();
      for (size_t i = 0; i < cloud3f.size(); i++){
	Eigen::Vector3f p = cloud3f[i];
	geometry_msgs::Point32 point;
	point.x = p.x();
	point.y = p.y();
	point.z = p.z();
	cloud.points.push_back(point);
      }
      cloud_pub.publish(cloud);
    }
    laser_pub.publish(scan);
    //...
}
void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& info){
    //K matrix
    //================================================================================
    camerainfo.K = info->K;
    ROS_INFO("Got camera info!");
    K(0,0) = camerainfo.K.c_array()[0];
    K(0,1) = camerainfo.K.c_array()[1];
    K(0,2) = camerainfo.K.c_array()[2];
    K(1,0) = camerainfo.K.c_array()[3];
    K(1,1) = camerainfo.K.c_array()[4];
    K(1,2) = camerainfo.K.c_array()[5];
    K(2,0) = camerainfo.K.c_array()[6];
    K(2,1) = camerainfo.K.c_array()[7];
    K(2,2) = camerainfo.K.c_array()[8];

    d2l.setK(K);//--------------------------------------

    cerr << K << endl;

    //TF
    tf::TransformListener* listener;
    listener = new tf::TransformListener();  // Starts a thread consuming a lot of CPU !!!
    //================================================================================
    ROS_INFO_STREAM("wait for camera tf " << c.base_frame_id << " - " << info->header.frame_id << " ...");
    tf::StampedTransform camera_tf;
    listener->waitForTransform(c.base_frame_id,info->header.frame_id,ros::Time(0),ros::Duration(30));
    listener->lookupTransform(c.base_frame_id, info->header.frame_id,ros::Time(0), camera_tf);
    camera_transform=tfTransform2eigen(camera_tf);
    ROS_INFO("got camera transform");
    cerr << camera_transform.matrix() << endl;
    
    ROS_INFO_STREAM("wait for laser tf " << c.base_frame_id << " - " << c.laser_frame_id << " ...");
    tf::StampedTransform laser_tf;
    listener->waitForTransform(c.base_frame_id,c.laser_frame_id,ros::Time(0),ros::Duration(30));
    listener->lookupTransform(c.base_frame_id, c.laser_frame_id,ros::Time(0), laser_tf);
    ROS_INFO("got laser transform");

    laser_transform=tfTransform2eigen(laser_tf);

    camera2laser_transform=laser_transform.inverse()*camera_transform;
    ROS_INFO("camera2laser transform");
    cerr << camera2laser_transform.matrix() << endl;
    
    d2l.setCamera2Laser(camera2laser_transform);//---------------------------

    //Don't want to receive any camera info stuff, basically this callback is used a init function
    info_sub.shutdown();

    delete listener;  // Kills the thread that consumes CPU !!!

}


//Params echo
//================================================================================
void EchoParameters(){
    cerr << "_angle_min: " <<  c.angle_min << endl;
    cerr << "_angle_max: " << c.angle_max << endl;
    cerr << "_num_ranges: " << c.num_ranges << endl;
    cerr << "_range_min: " << c.range_min << endl;
    cerr << "_range_max: " << c.range_max << endl;
    cerr << "_laser_plane_thickness: " << c.laser_plane_thickness << endl;
    cerr << "_base_frame_id: " << c.base_frame_id << endl;
    cerr << "_laser_frame_id: " << c.laser_frame_id << endl;
    cerr << "_camera_image_topic: " << c.camera_image_topic << endl;
    cerr << "_camera_info_topic: " << c.camera_info_topic << endl;
    cerr << "_pointcloud_topic: " << c.pointcloud_topic << endl;
    cerr << "_laser_topic: " <<  c.laser_topic << endl;
    cerr << "_publish_pointcloud: " << c.publish_pointcloud << endl;
    cerr << "_num_ranges: " <<  c.num_ranges << endl;

}


int main(int argc, char **argv){

    ros::init(argc, argv, "depth2laser");
    ros::NodeHandle n("~");


    //Getting and setting parameters
    n.param("angle_min", c.angle_min, -M_PI/2);
    n.param("angle_max", c.angle_max, M_PI/2);
    n.param("num_ranges", c.num_ranges, 1024);
    n.param("range_min", c.range_min, 0.1);
    n.param("range_max", c.range_max, 10.0);
    n.param("laser_plane_thickness", c.laser_plane_thickness, 0.05);
    n.param<string>("base_frame_id", c.base_frame_id, "/base_link");
    n.param<string>("laser_frame_id", c.laser_frame_id, "/laser_frame");
    n.param<string>("camera_image_topic", c.camera_image_topic, "/camera/depth/image_raw");
    n.param<string>("camera_info_topic", c.camera_info_topic, "/camera/depth/camera_info");
    n.param<string>("pointcloud_topic", c.pointcloud_topic, "/pointcloud");
    n.param<string>("laser_topic", c.laser_topic, "/scan");
    n.param("publish_pointcloud", c.publish_pointcloud, 0);

	Laser_config.angle_min=c.angle_min;
	Laser_config.angle_max=c.angle_max;
	Laser_config.num_ranges=c.num_ranges;
	Laser_config.range_min=c.range_min;
	Laser_config.range_max=c.range_max;
	Laser_config.laser_plane_thickness=c.laser_plane_thickness;
	d2l.setLaserConfig(Laser_config);

    EchoParameters();
    //Messages headers for TF
    //================================================================================
    clearScan();

    //Subscribers
    //================================================================================
    frame_sub = n.subscribe(c.camera_image_topic, 1, frameCallback);
    info_sub = n.subscribe(c.camera_info_topic, 1, infoCallback);
    //Publishers
    //================================================================================
    cloud_pub = n.advertise<sensor_msgs::PointCloud>(c.pointcloud_topic, 1);
    laser_pub = n.advertise<sensor_msgs::LaserScan>(c.laser_topic, 1);

    ros::spin();

    return 0;
}
