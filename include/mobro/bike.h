#include <mobro/robot.h>
#include <eigen3/Eigen/Core>
#include <mobro/mpc/mpc.h>

class Bike : public Robot
{  
  double L{1.6};
  double r{0.53};
  double beta_max{1.5};
  double beta;
  Float32MultiArray cmd;
  //std::shared_ptr<BicycleModel> model{std::make_shared<BicycleModel>(L)};
  //MobileMPC<4> mpc;

  inline std::array<double, 6> CS() const
  {
    return {cos(theta),sin(theta),cos(beta),sin(beta),cos(theta+beta),sin(theta+beta)};
  }

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

  void computeStaticFB(const Goal &goal)
  {
    const auto [c,s,cb,sb,ctb,stb] = CS(); {}
    Vector2d xyp = xy;
    xyp[0] += L*c + ctb*gains.d;
    xyp[1] += L*s + stb*gains.d;

    K(0,0) = ctb-gains.d/L*stb*sb;
    K(0,1) = -gains.d*stb;
    K(1,0) = stb+gains.d/L*ctb*sb;
    K(1,1) = gains.d*ctb;

    u = solve(K, goal.vd + gains.K*(goal.pd-xyp));
  }

  void computeDynamicFB(const Goal &goal)
  {
    const auto [c,s,cb,sb,ctb,stb] = CS(); {}
    Vector2d xyp = xy;
    xyp[0] += L*c;
    xyp[1] += L*s;

    Vector2d vp = linearVelocity(false);
    vp[0] -= u[0]*stb;
    vp[1] += u[0]*c*sb;

    K(0,0) = ctb;
    K(0,1) = -u[0]*ctb;
    K(1,0) = stb;
    K(1,1) = u[0]*ctb;

    const auto u_tilde{solve(K, goal.ad + gains.Kp*(goal.pd-xyp) + gains.Kv*(goal.vd-vp))};

    // to velocity
    u[0] += u_tilde[0]*dt;
    u[1] = u_tilde[1];
  }

  void computeLyapunovFB(const Goal &goal)
  {
    const auto [xe, ye, te, vref, wref] = LyapunovError(goal.pd, goal.vd, goal.ad, L, beta); {}
    u[0] = vref*cos(te) + gains.Kx*xe;
    u[1] = wref + gains.Ky*ye*vref*sinc(te) + gains.Kt*te - u[0]/L*sin(beta);
  }

  void computeMPC(const Goal &goal, const rclcpp::Time &now);

  void buildCmd(const rclcpp::Time &now)
  {
    u[0] = std::clamp(u[0], -gains.vmax, gains.vmax);
    u[1] = std::clamp(u[1], -gains.wmax, gains.wmax);

    // to cmd_vel
    cmd_vel.linear.x = u[0]*cos(beta);
    cmd_vel.angular.z = u[0]*sin(beta)/L;

    // to joints
    cmd.data[0] = u[0];
    cmd.data[1] = u[1];
    cmd_pub->publish(cmd);
  }



};
