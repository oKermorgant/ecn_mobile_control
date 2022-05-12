#include <mobro/robot.h>
#include <eigen3/Eigen/Core>
#include <ct/core/Systems>
#include <mobro/mpc/mpc.h>


class TwoSteering : public Robot
{  
  double L{1.};
  double r{0.4};
  double beta_max{1.};
  double beta1{}, beta2{};
  Float32MultiArray cmd;
  //std::shared_ptr<BicycleModel> model{std::make_shared<BicycleModel>(L)};
  //MobileMPC<4> mpc;
public:
  explicit TwoSteering(rclcpp::Node::SharedPtr node, const Traj &traj) : Robot(node, traj, 3)
  {
    js_sub = node->create_subscription<JointState>("robot/joint_states", 2, [&](const JointState::SharedPtr msg)
    {
      size_t i{};
      for(const auto &name: msg->name)
      {
        if(name == "front_wheel_steering")
          beta1 = msg->position[i];
        else if(name == "rear_wheel_steering")
          beta2 = msg->position[i];
        i++;
      }
    });
    cmd.data.resize(3);
    cmd_pub = node->create_publisher<Float32MultiArray>("robot/cmd", 1);

  }

  void update(const rclcpp::Time &now) override;
};
