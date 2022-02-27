#include <ros/ros.h>
#include <ecn_mobile_control/GainsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/JointState.h>

#include "traj.h"

using ecn_mobile_control::GainsConfig;
using sensor_msgs::JointState;

inline double sinc(double x)
{
  if(x == 0.) return 1.;
  return sin(x)/x;
}

inline double toPi(double t)
{
  return fmod(t+M_PI, 2*M_PI) - M_PI;
}

class Robot
{
protected:
  ros::NodeHandle & nh;
  const Traj & traj;
  ros::Publisher js_pub;
  sensor_msgs::JointState js;

  ros::Subscriber goal_sub;
  geometry_msgs::PoseStamped manual_goal, goal;
  bool manual_goal_used{false};
  ros::Publisher goal_pub;


  vpColVector xy, u;
  vpMatrix K;
  double theta{0};
  double vx{0}, vy{0}, w{0};

  double dt{0.05};

  GainsConfig gains;
  dynamic_reconfigure::Server<GainsConfig> dr_server;

  tf2_ros::TransformBroadcaster br;

  std::tuple<vpColVector, vpColVector, vpColVector> currentGoal(vpColVector xyp, const ros::Time &now)
  {
    auto [p,v,a] = traj.ref(now.toSec()); {}

    goal.header.stamp = now;
    goal.header.frame_id = "world";
    goal.pose.position.x = p[0];
    goal.pose.position.y = p[1];
    const auto theta_d{atan2(v[1], v[0])};
    goal.pose.orientation.w = cos(theta_d/2);
    goal.pose.orientation.z = sin(theta_d/2);

    static double t0{-1};
    if(!manual_goal_used || gains.lyapunov)
    {
      t0 = -1;
      goal_pub.publish(goal);
      return {p,v,a};
    }

    const auto dx{xyp[0]-manual_goal.pose.position.x};
    const auto dy{xyp[1]-manual_goal.pose.position.y};

    if(sqrt(dx*dx+dy*dy) < 1e-3)
    {
      if(t0 < 0) t0 = now.toSec();
      if((now.toSec()-t0) > 5)
      {
        manual_goal_used = false;
        goal_pub.publish(goal);
        return {p,v,a};
      }
    }

    p[0] = manual_goal.pose.position.x;
    p[1] = manual_goal.pose.position.y;
    v = 0;
    return {p,v,a};
  }

public:
  explicit Robot(ros::NodeHandle &nh, const Traj &traj) : nh{nh}, traj{traj}, js_pub{nh.advertise<sensor_msgs::JointState>("joint_states", 1)}
  {
    js.name = {"carrot", "carrot_disable"};
    js.position = {0, 0};
    dr_server.setCallback([&](const GainsConfig &config, int){this->gains = config;});

    goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, [&](geometry_msgs::PoseStampedConstPtr goal)
    {
      manual_goal = *goal;
      manual_goal_used = true;
      goal_pub.publish(goal);
    });

    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("goal",1);

    xy.resize(2);
    K.resize(2, 2);
    u.resize(2);
  }

  std::array<double, 5> LyapunovError(const vpColVector &p, const vpColVector &v, const vpColVector &a,
                                      double L=0, double beta=0) const
  {
    const auto c{cos(theta)};
    const auto s{sin(theta)};
    auto vref{c*v[0] + s*v[1]};
    double wref{};
    if(std::abs(vref) > 1e-3)
      wref = (v[0]*a[1] - v[1]*a[0])/(vref*vref);

    const auto dx{p[0]-xy[0] - L*c};
    const auto dy{p[1]-xy[1] - L*s};
    const auto xe{c*dx + s*dy};
    const auto ye{-s*dx + c*dy};
    const auto theta_goal{atan2(v[1], v[0])};
    const auto te{toPi(theta_goal-theta-beta)};
    return {xe, ye, te, vref, wref};
  }

  double rate() const {return 1./dt;}
  virtual void update(const ros::Time &now) = 0;

protected:

  void publishTF(const ros::Time &now)
  {
    geometry_msgs::TransformStamped tf;
    tf.header.frame_id = "world";
    tf.header.stamp = now;
    tf.transform.rotation.w = cos(theta/2);
    tf.transform.rotation.z = sin(theta/2);
    tf.transform.translation.x = xy[0];
    tf.transform.translation.y = xy[1];
    tf.child_frame_id = "footprint";
    br.sendTransform(tf);
  }

  void publish(const ros::Time &now)
  {    
    xy[0] += vx*dt;
    xy[1] += vy*dt;
    theta += w*dt;
    publishTF(now);

    js.header.stamp = ros::Time::now();
    if(gains.lyapunov)
    {
      js.position[1] = 99;
    }
    else
    {
      js.position[0] = gains.d;
      js.position[1] = 0;
    }
    js_pub.publish(js);
  }
};


class Unicycle : public Robot
{
public:
  explicit Unicycle(ros::NodeHandle &nh, const Traj &traj) : Robot(nh, traj)
  {

  }

  void update(const ros::Time &now) override
  {
    const auto c{cos(theta)};
    const auto s{sin(theta)};
    vpColVector xyp = xy;
    xyp[0] += c*gains.d;
    xyp[1] += s*gains.d;
    const auto [p,v,a] = currentGoal(xyp, now); {}

    // compute command
    if(gains.lyapunov)
    {
      const auto [xe, ye, te, vref, wref] = LyapunovError(p, v, a); {}

      u[0] = vref*cos(te) + gains.Kx*xe;
      u[1] = wref + gains.Ky*ye*vref*sinc(te) + gains.Kt*te;
    }
    else
    {
      K[0][0] = c;
      K[0][1] = -gains.d*s;
      K[1][0] = s;
      K[1][1] = gains.d*c;

      u = gains.Kp*K.solveBySVD(p-xyp) + v;
    }

    // update position
    vx = u[0]*c;
    vy = u[0]*s;
    w = u[1];

    publish(now);
  }
};


class Bike : public Robot
{
  double L{1.6};
  double r{0.53};
  double bdot_max{2.};
  double beta_max{1.5};

public:
  explicit Bike(ros::NodeHandle &nh, const Traj &traj) : Robot(nh, traj)
  {
    js.name.push_back("frame_to_handlebar");
    js.name.push_back("handlebar_to_frontwheel");
    js.name.push_back("frame_to_backwheel");
    js.position.resize(5, 0.);
  }

  void update(const ros::Time &now) override
  {
    const auto c{cos(theta)};
    const auto s{sin(theta)};
    const auto beta{js.position[2]};
    const auto cb{cos(beta)};
    const auto sb{sin(beta)};
    const auto ctb{cos(theta+beta)};
    const auto stb{sin(theta+beta)};

    vpColVector xyp = xy;
    xyp[0] += L*c + ctb*gains.d;
    xyp[1] += L*s + stb*gains.d;
    const auto [p,v,a] = currentGoal(xyp, now); {}

    // compute command
    if(gains.lyapunov)
    {
      const auto [xe, ye, te, vref, wref] = LyapunovError(p, v, a, L, beta); {}
      u[0] = vref*cos(te) + gains.Kx*xe;
      u[1] = wref + gains.Ky*ye*vref*sinc(te) + gains.Kt*te - u[0]/L*sb;
    }
    else
    {
      K[0][0] = ctb-gains.d/L*stb*sb;
      K[0][1] = -gains.d*stb;
      K[1][0] = stb+gains.d/L*ctb*sb;
      K[1][1] = gains.d*ctb;

      u = gains.Kp*K.solveBySVD(p-xyp) + v;
    }

    // update position
    vx = u[0]*c*cb;
    vy = u[0]*s*cb;
    w = u[0]*sb/L;

    // update wheels
    u[1] = std::clamp(u[1], -bdot_max, bdot_max);

    js.position[2] = std::clamp(js.position[2]+u[1]*dt, -beta_max, beta_max);
    js.position[3] += u[0]*dt/r;
    js.position[4] += (vx*c + vy*s)*dt/r;

    publish(now);
  }
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "control");
  ros::NodeHandle nh;
  const Traj traj;  

  std::unique_ptr<Robot> robot;
  //if(nh.param<std::string>("~robot", "bike") == "bike")
  {
    ROS_INFO("Using bicycle model");
    robot = std::make_unique<Bike>(nh, traj);
  }
 /* else
  {
    robot = std::make_unique<Unicycle>(nh, traj);
  }*/

  auto path_pub = nh.advertise<nav_msgs::Path>("path", 1);

  ros::Rate rate(robot->rate());
  while(ros::ok())
  {
path_pub.publish(traj.fullPath());
    robot->update(ros::Time::now());
    ros::spinOnce();
    rate.sleep();
  }

}
