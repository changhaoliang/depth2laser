// Aldebaran includes.
#include <alproxies/almotionproxy.h>
#include <qi/applicationsession.hpp>

// Boost includes.
#include <boost/program_options.hpp>

#include <thread>
#include <math.h>
#include <iostream>

using namespace std;
using namespace AL;
namespace po = boost::program_options;

struct HeadControlConfig {
  float head_yaw;
  float head_pitch;
};

std::thread ham_thread;
bool stop_thread = false;
HeadControlConfig config;

void head_angles_monitor_thread(qi::AnyObject motion_service) {
  cerr << "Starting head angles monitor thread" << endl;

  float threshold = 0.1;
  ALValue desired_angles = ALValue::array(config.head_yaw, config.head_pitch);
  ALValue angle_names    = ALValue::array("HeadYaw", "HeadPitch");
  
  while(!stop_thread){
    //Read head angles
    vector<float> current_head_angles = motion_service.call<vector<float>>("getAngles", angle_names, true);
    float current_head_yaw = current_head_angles[0];
    float current_head_pitch = current_head_angles[1];
    if(abs(current_head_yaw - config.head_yaw) >= threshold || abs(current_head_pitch - config.head_pitch) >= threshold ) {
      motion_service.call<void>("setAngles", angle_names, desired_angles, 0.2f);
    }
    usleep(200000); //200ms
  }

  std::cerr << "Head control finished. Setting head angles to 0,-0.2 (Default Stand posture)" << std::endl;
  motion_service.call<void>("setAngles", angle_names, ALValue::array(0, -0.2), 0.2f);
  
  std::cerr << "Thread exit successfully" << std::endl;
}

void head_control_enabled_callback(qi::AnyValue value, qi::AnyObject motion_service)
{
  bool head_control_enabled = value.as<bool>();
  if (head_control_enabled)
  {
    //Checking if thread was stopped before
    if (stop_thread) {
      //Starting thread
      stop_thread = false;
      ham_thread = std::thread(&head_angles_monitor_thread, motion_service);
    }
    cerr << "Head Angle Control Enabled" << endl;
  }
  else
  {
    stop_thread = true;
    cerr << "Head Angle Control Disabled" << endl;
    ham_thread.join();
  }
  return;
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
    ("head_yaw", po::value<float>()->default_value(0.0f), "Pepper's desired head yaw angle.")
    ("head_pitch", po::value<float>()->default_value(-0.2f), "Pepper's desired head pitch angle.")
    ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, description), vm);
  po::notify(vm);
  // --help option
  if (vm.count("help")){
    std::cout << description << std::endl;
    return 0;
  }

  //Params
  const std::string pip = vm["pip"].as<std::string>();
  int pport = vm["pport"].as<int>();
  config.head_yaw = vm["head_yaw"].as<float>();
  config.head_pitch = vm["head_pitch"].as<float>();

    if (pip == ""){
    std::cerr << "PEPPER_IP not defined. Please, set robot ip through program options" << std::endl;
    exit(0);
  }

  cerr << endl;
  cerr << "==========================" << endl;
  cerr << "pepper_ip: " << pip <<endl;
  cerr << "pepper_port: " << pport <<endl;
  cerr << "head_yaw: "<< config.head_yaw << endl;
  cerr << "head_pitch: "<< config.head_pitch << endl;
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

  //Init services
  qi::SessionPtr session = app.session();
  qi::AnyObject motion_service = session->service("ALMotion");
  qi::AnyObject memory_service = session->service("ALMemory");

  //Set stiffness head 1.0
  motion_service.call<void>("setStiffnesses", ALValue("Head"), 1.0f);

  //Subscribing to head control enable signal
  qi::AnyObject subscriber_head_control_enabled = memory_service.call<qi::AnyObject>("subscriber","PepperHeadControl/Enabled");
  qi::SignalLink signal_head_control_enabled_id = subscriber_head_control_enabled.connect("signal", (boost::function<void(qi::AnyValue)>(boost::bind(&head_control_enabled_callback, _1, motion_service))));
  
  //start thread
  ham_thread = std::thread(&head_angles_monitor_thread, motion_service);

  app.run();
  //Check if thread is running
  if (!stop_thread) {
    stop_thread = true;
    ham_thread.join();
  }
  std::cerr << "Pepper head angles monitor exit successfully" << std::endl;
}
