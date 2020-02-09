// Aldebaran includes.
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <qi/applicationsession.hpp>

// Boost includes.
#include <boost/program_options.hpp>
#include <depth2laser.h>
#include <iostream>
#include <chrono>

//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

// Opencv includes.
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <thread>

using namespace std;
using namespace cv;
using namespace AL;
namespace po = boost::program_options;

bool stop_thread = false;
std::vector<float> ranges;
Depth2Laser d2l;
LaserConfig laser_conf;
int num=0;
int rate;
float laser_z;
vector<Mat> images;

Eigen::Isometry3f setLaserTf(){
  Eigen::Isometry3f laser_tf = Eigen::Isometry3f::Identity();

  laser_tf.translation().z() = laser_z;
  /*Laser position
  //laser_tf.translation().x() = 0;
  //laser_tf.translation().y() = 0;

  //Laser rotation
  //============================================================
  //===the angle you want  the order is !!!!!!!!z y x!!!!!!!!===
  //============================================================
  Eigen::Matrix3f m = Eigen::AngleAxisf(0.100006826222, Eigen::Vector3f::UnitZ()).matrix()
  * Eigen::AngleAxisf(0.0232329126447,  Eigen::Vector3f::UnitY()).matrix()
  * Eigen::AngleAxisf(-0.20040678978, Eigen::Vector3f::UnitX()).matrix();
  Laser_tf.linear() = m;*/

  return laser_tf;
}

Eigen::Isometry3f getCameraPose(const qi::AnyObject motion_service, int frame){
  bool useSensorValues  = true;
  std::string name = "CameraDepth";
  std::vector<float> result = motion_service.call<std::vector<float>>("getTransform", name, frame, useSensorValues);

  // R R R x
  // R R R y
  // R R R z
  // 0 0 0 1
  Eigen::Isometry3f tf = Eigen::Isometry3f::Identity();
  tf.translation().x() = result[3];
  tf.translation().y() = result[7];
  tf.translation().z() = result[11];
  Eigen::Matrix3f R;

  R(0,0) = result[0];
  R(0,1) = result[1];
  R(0,2) = result[2];
  R(1,0) = result[4];
  R(1,1) = result[5];
  R(1,2) = result[6];
  R(2,0) = result[8];
  R(2,1) = result[9];
  R(2,2) = result[10];
  tf.linear() = R;

  //std::cerr << "CameraPose: " << tf.matrix() << std::endl;
  
  return tf;
}
  //subscribe to camera
void initCamera(ALVideoDeviceProxy &camproxy, std::string &clientName ){
  /** Subscribe a client image requiring 320*240 and BGR colorspace.*/
  clientName = "kDepthCamera" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  std::cerr << "Client name: " << clientName << std::endl;
  clientName = camproxy.subscribeCamera(clientName, 2, kQVGA, kDepthColorSpace, 30);
}


void getImageFromRemote(ALVideoDeviceProxy camProxy, std::string clientName, cv::Mat& image){
  ALValue img = camProxy.getImageRemote(clientName);
  image.data = (uchar*) img[6].GetBinary();
}

void getImageFromLocal(ALVideoDeviceProxy camProxy, std::string clientName, cv::Mat& image){
  ALImage* img = (ALImage*) camProxy.getImageLocal(clientName);
  image.data = img->getData();
}

void getImageFromFile(cv::Mat& image) {
  //read an image from disk
  if (num < images.size()){
    image = images[num++];
    cv::namedWindow("depthimage",CV_WINDOW_NORMAL);
    cv::imshow("depthimage",image);
    waitKey(1);
    if (num == images.size())
      stop_thread = true; //Loop will finish
  }
  return;
}

void visualizeLaser(const std::vector<float>& ranges){

  float resolution = 0.03;

  cv::Mat picture = cv::Mat(400,400,CV_8UC3,Scalar(255,255,255));

  //Drawing axis
  line(picture, cv::Point(0,200), cv::Point(400,200), Scalar(255,0,0));
  line(picture, cv::Point(200,0), cv::Point(200,400), Scalar(255,0,0));
  //Arrows
  line(picture, cv::Point(200,0), cv::Point(190,10), Scalar(255,0,0));
  line(picture, cv::Point(200,0), cv::Point(210,10), Scalar(255,0,0));
  line(picture, cv::Point(400,200), cv::Point(390,190), Scalar(255,0,0));
  line(picture, cv::Point(400,200), cv::Point(390,210), Scalar(255,0,0));
  putText(picture, "x", cv::Point(390,185), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,0));
  putText(picture, "y", cv::Point(215, 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,0));


  float angle_inc = (laser_conf.angle_max - laser_conf.angle_min)/laser_conf.num_ranges;
  for (int i=0; i<ranges.size(); i++){

    if(ranges[i]<laser_conf.range_max){
      float x, y;
      x = ranges[i]*cos(laser_conf.angle_min + i*angle_inc);
      y = ranges[i]*sin(laser_conf.angle_min + i*angle_inc);

      int u, v;
      u = (int)(1/resolution)*x;
      v = (int)(1/resolution)*(-y);
      cv::circle(picture, cv::Point(u+200,v+200), .5, cv::Scalar(0));
    }
  }
  cv::imshow("scan",picture);
  waitKey(1);
}

void publishLaser(const qi::AnyObject memory_service, const std::vector<float>& ranges){
  memory_service.call<void>("insertData","NAOqiDepth2Laser/Ranges",ranges);
  memory_service.call<void>("insertData","NAOqiDepth2Laser/NumRanges",laser_conf.num_ranges);
  memory_service.call<void>("insertData","NAOqiDepth2Laser/MinAngle",laser_conf.angle_min);
  memory_service.call<void>("insertData","NAOqiDepth2Laser/MaxAngle",laser_conf.angle_max);
  memory_service.call<void>("insertData","NAOqiDepth2Laser/MaxRange",laser_conf.range_max);
}

//cv::Mat& image
void depth2laser_thread(bool read_disk, ALVideoDeviceProxy camProxy, std::string clientName, int frame, Eigen::Isometry3f laser_tf, bool use_gui, bool remote, qi::AnyObject motion_service, qi::AnyObject memory_service){

  cv::Mat imgHeader = cv::Mat(cv::Size(320, 240), CV_16UC1);
  //std::cerr << "Matrix type: " << imgHeader.type() << std::endl;Ranges

  Eigen::Isometry3f camera_tf;
  Eigen::Isometry3f camera2laser_tf;
  Eigen::Isometry3f camera_depth_optical_frame;
  camera_depth_optical_frame(0,2) = 1;
  camera_depth_optical_frame(1,0) = -1;
  camera_depth_optical_frame(2,1) = -1;
  camera_depth_optical_frame(3,3) = 1;
  //std::cerr<<camera_depth_optical_frame.matrix();
  if (use_gui){
    cv::namedWindow("scan");
    cv::moveWindow("scan", 20, 20);

    cv::namedWindow("images");
    cv::moveWindow("images", 420, 20);
  }
  while (!stop_thread){
    std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
    //read pose of the camera
    camera_tf = getCameraPose(motion_service, frame);

    camera2laser_tf = laser_tf.inverse()*camera_tf*camera_depth_optical_frame;
    d2l.setCamera2Laser(camera2laser_tf);
    //std::cerr << "Camera2Laser: " << std::endl << camera2laser_tf.matrix() << std::endl;

    if(!read_disk)
    {
      if(!remote)
      {
        getImageFromLocal(camProxy, clientName, imgHeader);
      }
      else
      {
        getImageFromRemote(camProxy, clientName, imgHeader);
      }
    }
    else
    {
      getImageFromFile(imgHeader);
    }
    d2l.setImage(imgHeader);
    //compute
    d2l.compute();
    ranges = d2l.ranges();
    if (use_gui){
      visualizeLaser(ranges);
    }

    //publishing laser
    publishLaser(memory_service, ranges);

    //release image
    camProxy.releaseImage(clientName);

    //visualize
    if (use_gui){
      cv::normalize(imgHeader, imgHeader, 0, 65535, cv::NORM_MINMAX, CV_16UC1);
      cv::imshow("images", imgHeader);
      cv::waitKey(30); //ms
    }

    std::chrono::steady_clock::time_point time_end = std::chrono::steady_clock::now();
    int cycle_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();
    std::cerr << "Cycle " << cycle_ms << " milliseconds" << std::endl;
    if (cycle_ms < rate)
      usleep((rate-cycle_ms)*1e3);
    
  }
  //Cleanup
  camProxy.unsubscribe(clientName);
  std::cerr << "Thread exit successfully" << std::endl;
}

int main(int argc, char **argv)
{
  std::string pepper_ip = "";
  if (std::getenv("PEPPER_IP") != NULL)
    pepper_ip = std::getenv("PEPPER_IP");

  po::options_description description("Options");
  description.add_options()
    ("help", "Displays this help message")
    ("pip", po::value<std::string>()->default_value(pepper_ip), "Robot IP address. Set IP here or for convenience, define PEPPER_IP as environment variable. On robot or Local Naoqi: use '127.0.0.1'.")
    ("pport", po::value<int>()->default_value(9559), "Naoqi port number.")
    ("angle_min", po::value<double>()->default_value(-M_PI/2), "Angle min for simulated scan.")
    ("angle_max", po::value<double>()->default_value(M_PI/2), "Angle max for simulated scan.")
    ("num_ranges", po::value<int>()->default_value(1024), "Num ranges for simulated scan.")
    ("range_min", po::value<double>()->default_value(0.1), "Range min for simulated scan.")
    ("range_max", po::value<double>()->default_value(10.0), "Range max for simulated scan.")
    ("laser_plane_thickness", po::value<double>()->default_value(0.05), "Laser plane thickness to consider points from depth image.")
    ("frame", po::value<int>()->default_value(2), "Frame with respect to which express position: (0 = Torso, 1 = World, 2 = Robot)")
    ("use_gui", po::value<bool>()->default_value(false), "Visualize images. Not possible when running in Pepper.")
    ("remote", po::value<bool>()->default_value(true), "Get image remote.")
    ("read_disk", po::value<bool>()->default_value(false), "Read images from disk.")
    ("disk_path", po::value<cv::String>()->default_value(""), "Please, input your disk path to the images.")
    ("rate", po::value<int>()->default_value(100), "Laser scan publishing rate.")
    ("laser_z", po::value<float>()->default_value(1.5), "Height where to simulate the laser.")
    ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, description), vm);
  po::notify(vm);
  // --help option
  if (vm.count("help")){
    std::cout << description << std::endl;
    return 0;
  }

  const std::string pip = vm["pip"].as<std::string>();
  int pport = vm["pport"].as<int>();
  double angle_min = vm["angle_min"].as<double>();
  double angle_max = vm["angle_max"].as<double>();
  int num_ranges = vm["num_ranges"].as<int>();
  double range_min = vm["range_min"].as<double>();
  double range_max = vm["range_max"].as<double>();
  double laser_plane_thickness = vm["laser_plane_thickness"].as<double>();
  laser_z = vm["laser_z"].as<float>();
  int frame = vm["frame"].as<int>();
  bool use_gui = vm["use_gui"].as<bool>();
  bool remote = vm["remote"].as<bool>();
  bool read_disk = vm["read_disk"].as<bool>();
  std::string images_path = vm["disk_path"].as<std::string>();
  cv::String disk_path = images_path + "/*.png";
  rate = vm["rate"].as<int>();

  if (pip == ""){
    std::cerr << "PEPPER_IP not defined. Please, set robot ip through program options" << std::endl;
    exit(0);
  }

  cerr << endl;
  cerr << "==========================" << endl;
  cerr << "pepper_ip: " << pip <<endl;
  cerr << "pepper_port: " << pport <<endl;
  cerr << "angle_min: "<< angle_min << endl;
  cerr << "angle_max: "<< angle_max << endl;
  cerr << "num_ranges: " << num_ranges << endl;
  cerr << "range_min: " << range_min << endl;
  cerr << "range_max: " << range_max << endl;
  cerr << "laser_plane_thickness: " << laser_plane_thickness << endl;
  cerr << "laser_z: " << laser_z << endl;
  cerr << "frame: " << frame <<endl;
  cerr << "use gui: " << use_gui << endl;
  cerr << "remote: " << remote << endl;
  cerr << "read_disk: " << read_disk << endl;
  cerr << "rate: " << rate << endl;
  cerr << "==========================" << endl << endl;

  std::string tcp_url("tcp://"+pip+":"+std::to_string(pport));

  qi::ApplicationSession app(argc, argv, 0, tcp_url);
  try {
    app.startSession();
  }
  catch (qi::FutureUserException e) {
    std::cerr << "Connection refused." << std::endl;
    exit(1);
  }

  //Init services and camera
  qi::SessionPtr session = app.session();
  qi::AnyObject motion_service = session->service("ALMotion");
  qi::AnyObject memory_service = session->service("ALMemory");

  /** Create a proxy to ALVideoDevice on the robot.*/
  ALVideoDeviceProxy camProxy(pip, pport);
  std::string clientName;
  initCamera(camProxy, clientName);

  //initialize some parameters (camera matrix and laser config)
  Eigen::Matrix3f K;
  K(0,0) = 525/2.0f;
  K(0,1) = 0;
  K(0,2) = 319.5000000/2.0f;
  K(1,0) = 0;
  K(1,1) = 525/2.0f;
  K(1,2) = 239.5000000000000/2.0f;
  K(2,0) = 0;
  K(2,1) = 0;
  K(2,2) = 1;

  d2l.setK(K);
  laser_conf.angle_max = angle_max;
  laser_conf.angle_min = angle_min;
  laser_conf.num_ranges = num_ranges;
  laser_conf.range_max = range_max;
  laser_conf.range_min = range_min;
  laser_conf.laser_plane_thickness = laser_plane_thickness;
  d2l.setLaserConfig(laser_conf);

  //save images from disk into images
  if (read_disk == 1) {
    if (images_path == ""){
      std::cerr << "Please, provide a path to the images" << std::endl;
      exit(0);
    }
    else {
      vector<cv::String> fn;
      cv::glob(disk_path, fn, true);
      for (int i = 0; i < fn.size(); i++)
	images.push_back(imread(fn[i],CV_LOAD_IMAGE_ANYDEPTH));
      std::cerr << "Found " << fn.size() << " images" << std::endl;
    }
  }

  //get laser tf by predefined
  Eigen::Isometry3f laser_tf;
  laser_tf = setLaserTf();

  std::thread d2l_thread = std::thread(&depth2laser_thread, read_disk, camProxy, clientName, frame, laser_tf, use_gui, remote, motion_service, memory_service);

  app.run();
  stop_thread = true;
  d2l_thread.join();
  std::cerr << "Depth2laser exit successfully" << std::endl;
}
