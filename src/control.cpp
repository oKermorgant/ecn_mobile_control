#include <ros/ros.h>
#include <ecn_mobile_control/GainsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/JointState.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include "traj.h"

using namespace ecn_mobile_control;
using sensor_msgs::JointState;
using std_msgs::Float64;
using std_msgs::Float64MultiArray;

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
  ros::Publisher goal_pub, error_pub, norm_pub;

  vpColVector xy, u, v;
  vpMatrix K;
  double theta{0}, w{0};

  double dt{0.05};

  GainsConfig gains;
  dynamic_reconfigure::Server<GainsConfig> dr_server;

  tf2_ros::TransformBroadcaster br;

  std::tuple<vpColVector, vpColVector, vpColVector> currentGoal(const ros::Time &now)
  {
    auto [pd,vd,ad] = traj.ref(now.toSec()); {}

    goal.header.stamp = now;
    goal.header.frame_id = "world";
    goal.pose.position.x = pd[0];
    goal.pose.position.y = pd[1];
    const auto theta_d{atan2(vd[1], vd[0])};
    goal.pose.orientation.w = cos(theta_d/2);
    goal.pose.orientation.z = sin(theta_d/2);

    static double t0{-1};     
    
    if(!manual_goal_used)
    {
      t0 = -1;
      goal_pub.publish(goal);
      return {pd,vd,ad};
    }

    if(v.frobeniusNorm() < 1e-3)
    {
      if(t0 < 0) t0 = now.toSec();
      if((now.toSec()-t0) > 5)
      {
        manual_goal_used = false;
        goal_pub.publish(goal);
        return {pd,vd,ad};
      }
    }
    pd[0] = manual_goal.pose.position.x;
    pd[1] = manual_goal.pose.position.y;
    vd = 0;
    return {pd,vd,ad};
  }    

public:
  explicit Robot(ros::NodeHandle &nh, const Traj &traj) : nh{nh}, traj{traj}, js_pub{nh.advertise<sensor_msgs::JointState>("joint_states", 1)}
  {
    js.name = {"carrot", "carrot_disable"};
    js.position = {0, 0};
    dr_server.setCallback([&](const GainsConfig &config, int)
    {
      gains = config;
      if(std::abs(gains.d) < 1e-3)  gains.d = 1e-3;
    });

    goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, [&](geometry_msgs::PoseStampedConstPtr goal)
    {
       manual_goal = *goal;
       manual_goal_used = true;
       goal_pub.publish(goal);
  });

    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("goal",1);

    xy.resize(2);
    v.resize(2);
    K.resize(2, 2);
    u.resize(2);


    error_pub = nh.advertise<Float64MultiArray>("error", 1);
    norm_pub = nh.advertise<std_msgs::Float64>("norm", 1);
  }

  std::array<double, 5> LyapunovError(const vpColVector &p, const vpColVector &v, const vpColVector &a,
                                      double L=0, double beta=0) const
  {
    const auto c{cos(theta)};
    const auto s{sin(theta)};
    auto vref{c*v[0] + s*v[1]};
    double wref{0.};
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

  void publish(const vpColVector &pd, const ros::Time &now)
  {
    // error
    auto error{pd - xy};
    Float64MultiArray error_msg;
    error_msg.data = error.toStdVector();
    error_pub.publish(error_msg);

    Float64 norm_msg;
    norm_msg.data = error.frobeniusNorm();
    norm_pub.publish(norm_msg);

    // update
    xy[0] += v[0]*dt;
    xy[1] += v[1]*dt;
    theta += w*dt;
    publishTF(now);    

    js.header.stamp = ros::Time::now();
    if(gains.control != Gains_FirstOrder)
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

    const auto [pd,vd,ad] = currentGoal(now); {}

    // compute command
    if(gains.control == Gains_Lyapunov)
    {
      const auto [xe, ye, te, vref, wref] = LyapunovError(pd, vd, ad); {}

      u[0] = vref*cos(te) + gains.Kx*xe;
      u[1] = wref + gains.Ky*ye*vref*sinc(te) + gains.Kt*te;
    }
    else if(gains.control == Gains_SecondOrder)
    {
      K[0][0] = c;
      K[0][1] = -v[1];
      K[1][0] = s;
      K[1][1] = v[0];

      // acceleration command
      const auto u_tilde{K.pseudoInverse()*(ad + gains.Kp*(pd-xy) + gains.Kv*(vd-v))};

      // to velocity
      u[0] += u_tilde[0]*dt;
      u[1] = u_tilde[1];
    }
    else  // first order
    {
      vpColVector xyp = xy;
      xyp[0] += c*gains.d;
      xyp[1] += s*gains.d;

      K[0][0] = c;
      K[0][1] = -gains.d*s;
      K[1][0] = s;
      K[1][1] = gains.d*c;

      u = K.solveBySVD(vd + gains.Kp*(pd-xyp));
    }

    u[0] = std::clamp(u[0], -gains.vmax, gains.vmax);
    u[1] = std::clamp(u[1], -gains.wmax, gains.wmax);

    // update position
    v[0] = u[0]*c;
    v[1] = u[0]*s;
    w = u[1];

    publish(pd, now);
  }
};


class Bike : public Robot
{
  double L{1.6};
  double r{0.53};
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

    const auto [pd,vd,ad] = currentGoal(now); {}

    // compute command
    if(gains.control == Gains_Lyapunov)
    {
      const auto [xe, ye, te, vref, wref] = LyapunovError(pd, vd, ad, L, beta); {}
      u[0] = vref*cos(te) + gains.Kx*xe;
      u[1] = wref + gains.Ky*ye*vref*sinc(te) + gains.Kt*te - u[0]/L*sb;
    }
    else if(gains.control == Gains_SecondOrder)
    {
      vpColVector xyp = xy;
      xyp[0] += L*c;
      xyp[1] += L*s;

      vpColVector vp = v;
      vp[0] -= u[0]*stb;
      vp[1] += u[0]*c*sb;

      K[0][0] = ctb;
      K[0][1] = -u[0]*ctb;
      K[1][0] = stb;
      K[1][1] = u[0]*ctb;

      const auto u_tilde{K.pseudoInverse()*(ad + gains.Kp*(pd-xyp) + gains.Kv*(vd-vp))};

      // to velocity
      u[0] += u_tilde[0]*dt;
      u[1] = u_tilde[1];
    }
    else
    {
      vpColVector xyp = xy;
      xyp[0] += L*c + ctb*gains.d;
      xyp[1] += L*s + stb*gains.d;

      K[0][0] = ctb-gains.d/L*stb*sb;
      K[0][1] = -gains.d*stb;
      K[1][0] = stb+gains.d/L*ctb*sb;
      K[1][1] = gains.d*ctb;

      u = K.solveBySVD(vd + gains.Kp*(pd-xyp));
    }

    u[0] = std::clamp(u[0], -gains.vmax, gains.vmax);

    // update position
    v[0] = u[0]*c*cb;
    v[1] = u[0]*s*cb;
    w = u[0]*sb/L;

    // update wheels
    u[1] = std::clamp(u[1], -gains.wmax, gains.wmax);

    js.position[2] = std::clamp(js.position[2]+u[1]*dt, -beta_max, beta_max);
    js.position[3] += u[0]*dt/r;
    js.position[4] += (v[0]*c + v[1]*s)*dt/r;

    publish(pd, now);
  }
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "control");
  ros::NodeHandle nh, priv("~");
  const Traj traj;

  std::unique_ptr<Robot> robot;
  if(priv.param<std::string>("robot", "uni") == "bike")
  {
    ROS_INFO("Using bicycle model");
    robot = std::make_unique<Bike>(nh, traj);
  }
  else
  {
    robot = std::make_unique<Unicycle>(nh, traj);
  }

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
