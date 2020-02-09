#include "depth2laser.h"
#include <iostream>
#include <math.h>
#include <vector>

// DEBUG Opencv includes.
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
Depth2Laser::Depth2Laser() {

    _camera2laser_transform = Eigen::Isometry3f::Identity();

    _laser_config.angle_max=0.0;
    _laser_config.angle_min=0.0;
    _laser_config.laser_plane_thickness=0.0;
    _laser_config.num_ranges=0;
    _laser_config.range_max=0.0;
    _laser_config.range_min=0.0;

    _got_info = false;
}

void Depth2Laser::compute()
{

  float angle_increment=(_laser_config.angle_max - _laser_config.angle_min)/_laser_config.num_ranges;
  float squared_max_norm = _laser_config.range_max*_laser_config.range_max;
  float squared_min_norm = _laser_config.range_min*_laser_config.range_min;
  float inverse_angle_increment = 1.0/angle_increment;
  int good_points = 0;

  _ranges.resize(_laser_config.num_ranges);
    
  //
  Eigen::Vector3f laser_point;
  Eigen::Vector3f camera_point;
  float theta;
  float range;
  int bin;
  //

  _cloud.clear();
  
  for(size_t i=0; i<_ranges.size();i++)
    _ranges[i] = _laser_config.range_max + 0.1;
    
  for(int i=0; i < _depth_image.rows; i++) {
    const ushort* row_ptr = _depth_image.ptr<ushort>(i);

    for(int j=0; j< _depth_image.cols; j++) {
      ushort id = row_ptr[j];
      if(id != 0) {
	float d = 1e-3*id;
	Eigen::Vector3f image_point(j*d,i*d,d);
	camera_point = _inv_K*image_point;
	laser_point = _camera2laser_transform*camera_point;

	_cloud.push_back(camera_point);
	if(fabs(laser_point.z()) < _laser_config.laser_plane_thickness) {

	  theta = atan2(laser_point.y(),laser_point.x());
	  range=laser_point.x()*laser_point.x()+laser_point.y()*laser_point.y();

	  if(range < squared_min_norm)
	    continue;
	  if(range > squared_max_norm)
	    continue;

	  bin=(int)((theta - _laser_config.angle_min)*inverse_angle_increment);

	  if(bin<0||bin>=_laser_config.num_ranges)
	    continue;

	  range=sqrt(range);
	  if(_ranges[bin]>range) {
	    _ranges[bin] = range;
	    good_points++;
	  }
	}
      }
    }
  }

  cerr<<good_points<<" ";
}

