#ifndef TRAJ_H
#define TRAJ_H

#include <visp/vpColVector.h>
#include <nav_msgs/Path.h>
#include <tuple>

struct Traj
{
  double w{.5};
  double a{3.};
  double b{2.};

  nav_msgs::Path fullPath() const
  {
    nav_msgs::Path path;
    path.header.frame_id = "world";
    auto t{-M_PI/w};
    const auto dt{0.1*M_PI};

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.header.stamp = ros::Time::now();
    pose.pose.orientation.w = 1.;

    while(t < M_PI/w)
    {
      const auto [p,v,acc] = ref(t); {}
      pose.pose.position.x = p[0];
      pose.pose.position.y = p[1];
      t += dt;
      path.poses.push_back(pose);
    }
    return path;
  }

  std::tuple<vpColVector, vpColVector, vpColVector> ref(double t) const
  {
    const auto c{cos(w*t)};
    const auto s{sin(w*t)};

    vpColVector pd(2), vd(2), ad(2);

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
