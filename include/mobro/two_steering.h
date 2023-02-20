#include <mobro/robot.h>
#include <eigen3/Eigen/Core>
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

  inline std::array<double, 10> CS() const
  {
    return {cos(theta),sin(theta),cos(beta1),sin(beta1),cos(beta2),sin(beta2),
          cos(theta + beta1),sin(theta + beta1),cos(theta + beta2),sin(theta + beta2)};
  }


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

  void computeStaticFB(const Goal &goal);

  void buildCmd(const rclcpp::Time &now)
  {
    const auto c1{cos(beta1)};
    const auto c2{cos(beta2)};
    const auto s1{sin(beta1)};
    const auto s2{sin(beta2)};
    u[0] = std::clamp(u[0], -gains.vmax, gains.vmax);
    u[1] = std::clamp(u[1], -gains.wmax, gains.wmax);
    u[2] = std::clamp(u[2], -gains.wmax, gains.wmax);

    const auto v2{u[0]*c1/c2};

    // to cmd vel
    cmd_vel.linear.x = u[0]*c1;
    cmd_vel.linear.y = v2*s2;
    cmd_vel.angular.z = (u[0]*s1-v2*s2)/L;

    // to joints
    cmd.data[0] = u[0];
    cmd.data[1] = u[1];
    cmd.data[2] = u[2];
    cmd_pub->publish(cmd);
  }



};
