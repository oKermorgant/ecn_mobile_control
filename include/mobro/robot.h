#ifndef MOBRO_ROBOT_H
#define MOBRO_ROBOT_H

#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <mobro/traj.h>
#include <mobro/utils.h>
#include <Eigen/Dense>


using sensor_msgs::msg::JointState;
using std_msgs::msg::Float32;
using std_msgs::msg::Float32MultiArray;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using rcl_interfaces::msg::SetParametersResult;

inline double sinc(double x)
{
  if(x == 0.) return 1.;
  return sin(x)/x;
}

inline double toPi(double t)
{
  if(t > M_PI)  return toPi(t-2*M_PI);
  if(t < -M_PI) return toPi(t+2*M_PI);
  return t;
}

enum class ControlType{STATIC_FB, DYNAMIC_FB, LYAPUNOV, MPC};

struct Gains
{
  // general
  ControlType control;

  double vmax, wmax;

  // static fb
  double d, K;
  // dynamic fb
  double Kp, Kv;
  // Lyapunov
  double Kx, Ky, Kt;
  // MPC
  double horizon, dt;
};

struct Goal
{
  Vector2d pd,vd,ad;
  bool use_traj;
};


class Robot
{        
protected:
  rclcpp::Node* node;
  const Traj & traj;
  rclcpp::Publisher<JointState>::SharedPtr carrot_pub;
  JointState carrot;
  rclcpp::Subscription<JointState>::SharedPtr js_sub;

  rclcpp::Subscription<PoseStamped>::SharedPtr goal_sub;
  geometry_msgs::msg::PoseStamped manual_goal, goal;
  bool manual_goal_used{false};

  rclcpp::Publisher<PoseStamped>::SharedPtr goal_pub;
  //ros::Publisher goal_pub, error_pub, norm_pub;

  // sim i/o
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub;
  Twist cmd_vel;
  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr cmd_pub;

  Vector2d xy;
  Eigen::VectorXd u;
  Eigen::MatrixXd K;
  double theta{0};

  double dt{0.05};
  int dt_ms = dt * 1000 - 2;

  Goal currentGoal(const rclcpp::Time &now);

  // robot-specific
  virtual void computeStaticFB(const Goal &goal) = 0;
  virtual void computeDynamicFB(const Goal &goal) {u.setZero();}
  virtual void computeLyapunovFB(const Goal &goal) {u.setZero();}
  virtual void computeMPC(const Goal &goal, const rclcpp::Time &now) {u.setZero();}

  virtual void buildCmd(const rclcpp::Time &now) = 0;

public:
  explicit Robot(rclcpp::Node::SharedPtr node, const Traj &traj, size_t dim_u);

  std::array<double, 5> LyapunovError(const Eigen::Vector2d &p, const Eigen::Vector2d &v, const Eigen::Vector2d &a,
                                      double L=0, double beta=0) const;

  inline std::chrono::milliseconds samplingTime() const {return std::chrono::milliseconds{(int)(1000*dt)};}


  void update(const rclcpp::Time &now);

  static inline Eigen::VectorXd solve(const Eigen::MatrixXd &K, const Eigen::VectorXd &v)
  {
    return K.colPivHouseholderQr().solve(v);
  }

  static std::string name(rclcpp::Node::SharedPtr node);

  inline void setVmax(double v, double w)
  {
    gains.vmax = v;
    gains.wmax = w;
  }

protected:

  inline Vector2d linearVelocity(bool relative = true) const
  {
    if(relative)
      return {cmd_vel.linear.x, cmd_vel.linear.y};
    const auto c{cos(theta)};
    const auto s{sin(theta)};
    return {c*cmd_vel.linear.x + s*cmd_vel.linear.y, s*cmd_vel.linear.x - c*cmd_vel.linear.y};
  }

  void publish(const Vector2d &pd, const rclcpp::Time &now);

  // gains stuff
  Gains gains;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_change;

  void declareParams();
  inline SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter> &parameters);
  inline ControlType declareParam(std::string name, std::map<ControlType, std::string> values);
  inline double declareParam(std::string name, double default_value, double lower, double upper);
};

#endif
