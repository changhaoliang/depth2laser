// Aldebaran includes.
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <qi/applicationsession.hpp>
#include <qi/session.hpp>
// Boost includes.
#include <boost/program_options.hpp>
#include <depth2laser.h>

//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <chrono>
#include <thread>
#include <iostream>

using namespace std;
using namespace cv;
using namespace AL;
namespace po = boost::program_options;

bool stop_thread = false;

void depth2laser_thread_gui(qi::AnyObject memory_service)
{
  while(!stop_thread)
  {
    float resolution = 0.03;

    qi::AnyValue subscribe_ranges = memory_service.call<qi::AnyValue>("getData","NAOqiDepth2Laser/Ranges");
    qi::AnyValue subscribe_num_ranges = memory_service.call<qi::AnyValue>("getData","NAOqiDepth2Laser/NumRanges");
    qi::AnyValue subscribe_angle_min = memory_service.call<qi::AnyValue>("getData","NAOqiDepth2Laser/MinAngle");
    qi::AnyValue subscribe_angle_max = memory_service.call<qi::AnyValue>("getData","NAOqiDepth2Laser/MaxAngle");
    qi::AnyValue subscribe_range_max = memory_service.call<qi::AnyValue>("getData","NAOqiDepth2Laser/MaxRange");

    const std::vector<float> ranges = subscribe_ranges.toList<float>();
    int num_ranges = subscribe_num_ranges.asInt32();
    float angle_min = subscribe_angle_min.asFloat();
    float angle_max = subscribe_angle_max.asFloat();
    float range_max = subscribe_range_max.asFloat();


    cv::Mat picture = cv::Mat(400,400,CV_8UC3,Scalar(255,255,255));

    //Drawing axis
    line(picture,cv::Point(0,200),cv::Point(400,200),Scalar(255,0,0));
    line(picture,cv::Point(200,0),cv::Point(200,400),Scalar(255,0,0));
    //Arrows
    line(picture, cv::Point(200,0), cv::Point(190,10), Scalar(255,0,0));
    line(picture, cv::Point(200,0), cv::Point(210,10), Scalar(255,0,0));
    line(picture, cv::Point(400,200), cv::Point(390,190), Scalar(255,0,0));
    line(picture, cv::Point(400,200), cv::Point(390,210), Scalar(255,0,0));
    putText(picture, "x", cv::Point(390,185), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,0));
    putText(picture, "y", cv::Point(215, 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,0));


    float angle_inc = (angle_max - angle_min)/num_ranges;

    for (int i = 0; i < ranges.size(); i++)
    {
      if(ranges[i] < range_max)
      {
        float x, y;
        x = ranges[i] * cos(angle_min + i * angle_inc);
        y = ranges[i] * sin(angle_min + i * angle_inc);

        int u, v;
        u = (int)(1/resolution) * x;
        v = (int)(1/resolution) * (-y);

        cv::circle(picture, cv::Point(u+200,v+200), 1, cv::Scalar(0));
      }
    }
    cv::imshow("scan",picture);
    waitKey(1);
  }
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

  if (pip == ""){
    std::cerr << "PEPPER_IP not defined. Please, set robot ip through program options" << std::endl;
    exit(0);
  }

  cerr << endl;
  cerr << "==========================" << endl;
  cerr << "pepper_ip: " << pip <<endl;
  cerr << "pepper_port: " << pport <<endl;
  cerr << "==========================" << endl << endl;

  std::string tcp_url("tcp://"+pip+":"+std::to_string(pport));

  qi::ApplicationSession app(argc, argv, 0, tcp_url);
  try
  {
    app.startSession();
  }
  catch (qi::FutureUserException e) {
    std::cerr << "Connection refused." << std::endl;
    exit(1);
  }
  qi::SessionPtr session = app.session();
  qi::AnyObject memory_service = session->service("ALMemory");

  std::thread d2l_thread_gui = std::thread(&depth2laser_thread_gui, memory_service);
  app.run();
  stop_thread = true;
  d2l_thread_gui.join();
}
