#ifndef TRAJ_H
#define TRAJ_H

#include <Eigen/Core>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/time.hpp>
#include <tuple>

using Eigen::Vector2d;
using nav_msgs::msg::Path;

struct Traj
{
  double w{.5};
  double a{3.};
  double b{2.};

  inline Path fullPath(const rclcpp::Time &now) const
  {
    static Path path;

    if(path.poses.empty())
    {
      path.header.frame_id = "map";
      auto t{-M_PI/w};
      const auto dt{0.1*M_PI};

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.orientation.w = 1.;

      while(t < M_PI/w)
      {
        const auto [p,v,acc] = ref(t); {}
        pose.pose.position.x = p[0];
        pose.pose.position.y = p[1];
        t += dt;
        path.poses.push_back(pose);
      }
    }

    for(auto &pose: path.poses)
      pose.header.stamp = now;

    return path;
  }

  inline Vector2d xy(double t) const
  {
    const auto c{cos(w*t)};
    const auto s{sin(w*t)};
    return {(a+b*c)*c, (a+b*c)*s};
  }

  std::tuple<Vector2d, Vector2d, Vector2d> ref(double t) const
  {
    const auto c{cos(w*t)};
    const auto s{sin(w*t)};

    Vector2d pd, vd, ad;

    pd[0] = (a+b*c)*c;
    pd[1] = (a+b*c)*s;

    vd[0] = -w*(a + 2*b*c)*s;
    vd[1] = w*(a*c - 2*b*s*s + b);

    ad[0] =  w*w*(-a*c + 4*b*s*s - 2*b);
    ad[1] =  -w*w*(a + 4*b*c)*s;

    return {pd,vd,ad};
  }
};



#endif // TRAJ_H
