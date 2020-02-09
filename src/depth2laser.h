#pragma once
#include <vector>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

struct LaserConfig {
  float angle_min;
  float angle_max;
  int num_ranges;
  float range_min;
  float range_max;
  float laser_plane_thickness;
};

typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > PointCloud3f;
  
class Depth2Laser {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Depth2Laser();

  //Set and get variables
  inline void setK(const Eigen::Matrix3f& K_) {
      _K = K_;
      _inv_K = K_.inverse();
      _got_info = true;
  }
  inline Eigen::Matrix3f K() {
      return _K;
  }

  inline void setCamera2Laser(const Eigen::Isometry3f& camera2laser_transform_) {
      _camera2laser_transform = camera2laser_transform_;
  }
  inline Eigen::Isometry3f camera2Laser(){
      return _camera2laser_transform;
  }

  inline void setLaserConfig(const LaserConfig laser_config_){
      _laser_config=laser_config_;
  }
  inline LaserConfig laserconfig(){
      return _laser_config;
  }

  inline void setImage(const cv::Mat& depth_image_){
      _depth_image=depth_image_;
  }
  inline cv::Mat& depthimage() {
      return _depth_image;
  }
  inline std::vector<float>& ranges()
  {
      return _ranges;
  }
  inline PointCloud3f& cloud() {return _cloud;}
  
  inline bool gotInfo() {return _got_info;}
  //Computes de virtual scan from the depth image
  void compute();
  
 protected:

  //Camera variables
  Eigen::Matrix3f _K;//
  Eigen::Matrix3f _inv_K;//
  Eigen::Isometry3f _camera2laser_transform;//

  bool _got_info;
  
  //Laser variables
  LaserConfig _laser_config;
  
  //Input
  cv::Mat _depth_image;

  //Output
  std::vector<float> _ranges;  
  PointCloud3f _cloud;
  
};
