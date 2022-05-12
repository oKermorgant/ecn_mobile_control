#include <mobro/robot.h>
#include <eigen3/Eigen/Core>
#include <ct/core/Systems>
#include <mobro/mpc/mpc.h>

class BicycleModel : public ct::core::ControlledSystem<4, 2>
{
  double L{};
public:
  explicit BicycleModel(double L) : L{L} {}
  inline ct::core::ControlledSystem<4,2>* clone() const override
  {
    return new BicycleModel(*this);
  }

  inline void computeControlledDynamics(const ct::core::StateVector<4>& state,
          [[maybe_unused]] const time_t& t,
          const ct::core::ControlVector<2>& control,
          ct::core::StateVector<4>& derivative) override
  {
    const auto c{cos(state(2))};
    const auto s{sin(state(2))};
    const auto cb{cos(state(3))};
    const auto sb{sin(state(3))};
    const auto v{control(0)};

    derivative(0) = v*c*cb;
    derivative(1) = v*s*cb;
    derivative(2) = v*sb/L;
    derivative(3) = control(1);
  }
};


class Bike : public Robot
{  
  double L{1.6};
  double r{0.53};
  double beta_max{1.5};
  double beta;
  Float32MultiArray cmd;
  //std::shared_ptr<BicycleModel> model{std::make_shared<BicycleModel>(L)};
  //MobileMPC<4> mpc;
public:
  explicit Bike(rclcpp::Node::SharedPtr node, const Traj &traj) : Robot(node, traj, 2)
  {
    js_sub = node->create_subscription<JointState>("robot/joint_states", 2, [&](const JointState::SharedPtr msg)
    {
      size_t i{};
      for(const auto &name: msg->name)
      {
        if(name == "frame_to_handlebar")
          beta = msg->position[i];
        i++;
      }
    });

    cmd.data.resize(2);
    cmd_pub = node->create_publisher<Float32MultiArray>("robot/cmd", 1);
  }

  void update(const rclcpp::Time &now) override;
};
