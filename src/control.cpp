#include <mobro/robot.h>
#include <mobro/unicycle.h>
#include <mobro/bike.h>
#include <mobro/two_steering.h>
#include <rclcpp/rclcpp.hpp>

using namespace std;
using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node{std::make_shared<rclcpp::Node>("control")};
  const Traj traj;

  const auto name{Robot::name(node)};

  std::unique_ptr<Robot> robot;
  if(name == "bike")
  {
    robot = std::make_unique<Bike>(node, traj);
  }
  else if(name == "two_steering")
  {
    robot = std::make_unique<TwoSteering>(node, traj);
  }
  else
  {
    robot = std::make_unique<Unicycle>(node, traj);
  }

  auto path_pub = node->create_publisher<Path>("path", 1);
  auto path_timer = node->create_wall_timer(1s, [&]()
  {
    path_pub->publish(traj.fullPath(node->get_clock()->now()));
  });

  auto robot_timer = node->create_wall_timer(robot->samplingTime(), [&]()
  {
    robot->update(node->get_clock()->now());
  });

  rclcpp::spin(node);
}
