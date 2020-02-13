#include <ecn_mobile_control/GainsConfig.h>
#include <dynamic_reconfigure/Config.h>
#include <ros/ros.h>

class Listener
{
public:
  Listener(ros::NodeHandle &nh)
  {
    gain_sub = nh.subscribe("/control/parameter_updates", 10, &Listener::gains_callback, this);
  }


private:
  ros::Subscriber gain_sub;
  std::map<std::string, double> gains;

  void gains_callback(const dynamic_reconfigure::ConfigConstPtr &msg)
  {
    std::cout << "Updating gains" << std::endl;
    for(const auto &elem: msg->doubles)
      gains[elem.name] = elem.value;

    std::cout << "Kt is now " << gains["Kt"] << std::endl;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gains_client_node");
  ros::NodeHandle nh;
  Listener listener(nh);

  ros::spin();





}
