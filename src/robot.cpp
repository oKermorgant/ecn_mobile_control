#include <mobro/robot.h>
#include <urdf/model.h>
#include <rclcpp/rclcpp.hpp>

Goal Robot::currentGoal(const rclcpp::Time &now)
{

  auto [pd,vd,ad] = traj.ref(now.seconds()); {}

  goal.header.stamp = now;
  goal.header.frame_id = "map";
  goal.pose.position.x = pd[0];
  goal.pose.position.y = pd[1];
  const auto theta_d{atan2(vd[1], vd[0])};
  goal.pose.orientation.w = cos(theta_d/2);
  goal.pose.orientation.z = sin(theta_d/2);

  static double t0{-1};

  if(!manual_goal_used)
  {
    t0 = -1;
    goal_pub->publish(goal);
    return {pd,vd,ad,true};
  }

  if(linearVelocity().norm() < 1e-3)
  {
    if(t0 < 0) t0 = now.seconds();
    if((now.seconds()-t0) > 5)
    {
      manual_goal_used = false;
      goal_pub->publish(goal);
      return {pd,vd,ad,true};
    }
  }
  pd[0] = manual_goal.pose.position.x;
  pd[1] = manual_goal.pose.position.y;
  vd.setZero();
  return {pd,vd,ad,false};
}

Robot::Robot(rclcpp::Node::SharedPtr node, const Traj &traj, size_t dim_u) :
  node{node.get()},
  traj{traj}
{
  u.resize(dim_u);
  K.resize(dim_u, dim_u);

  carrot_pub = node->create_publisher<JointState>("robot/joint_states", 1);

  carrot.name = {"carrot", "carrot_disable"};
  carrot.position = {0, 0};

  goal_sub = node->create_subscription<PoseStamped>("/move_base_simple/goal", 1, [&](const PoseStamped &goal)
  {
    manual_goal = goal;
    manual_goal_used = true;
    goal_pub->publish(goal);
  });

  goal_pub = node->create_publisher<PoseStamped>("goal",1);

  odom_sub = node->create_subscription<Odometry>("robot/odom", 1, [&](const Odometry &odom)
  {
    xy[0] = odom.pose.pose.position.x;
    xy[1] = odom.pose.pose.position.y;
    theta = 2*atan2(odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
  });
  cmd_vel_pub = node->create_publisher<Twist>("robot/cmd_vel", 1);

  //error_pub = node.advertise<Float64MultiArray>("error", 1);
  //norm_pub = node.advertise<std_msgs::Float64>("norm", 1);

  declareParams();
  param_change = node->add_on_set_parameters_callback(std::bind(&Robot::onParameterChange, this, std::placeholders::_1));
}

void Robot::declareParams()
{
  gains.control = declareParam("control", {{ControlType::STATIC_FB, "static fb"},
                                           {ControlType::DYNAMIC_FB, "dynamic fb"},
                                           {ControlType::LYAPUNOV, "Lyapunov"},
                                           {ControlType::MPC, "MPC"}});

  gains.vmax = declareParam("vmax", 3., 0., 10.);
  gains.wmax = declareParam("wmax", 2., 0., 10);

  // static fb
  gains.d = declareParam("static.d", 0.1, -5., 15.);
  gains.K = declareParam("static.Kp", 5., 0., 20.);
  // dynamic fb
  gains.Kp = declareParam("dynamic.Kp", 5., 0., 20.);
  gains.Kv = declareParam("dynamic.Kv", .5, 0., 20.);
  // Lyapunov
  gains.Kx = declareParam("Lyapunov.Kx", .5, 0., 20.);
  gains.Ky = declareParam("Lyapunov.Ky", .5, 0., 20.);
  gains.Kt = declareParam("Lyapunov.Kt", .5, 0., 20.);
  // MPC
  gains.horizon = declareParam("mpc.horizon", 1, 0., 5.);
  gains.dt = declareParam("mpc.dt", 0.1, 0.0, 1.);
}


SetParametersResult Robot::onParameterChange(const std::vector<rclcpp::Parameter> &parameters)
{
  for(auto &param: parameters)
  {
    auto &name{param.get_name()};
    if(name == "control")
    {
      gains.control = static_cast<ControlType>(param.as_int());
      continue;
    }
    const auto val{param.as_double()};
    if(name == "vmax") gains.vmax = val;
    else if(name == "wmax") gains.wmax = val;
    else if(name == "static.d") gains.d = val;
    else if(name == "static.Kp") gains.K = val;
    else if(name == "dynamic.Kp") gains.Kp = val;
    else if(name == "dynamic.Kv") gains.Kv = val;
    else if(name == "Lyapunov.Kx") gains.Kx = val;
    else if(name == "Lyapunov.Ky") gains.Ky = val;
    else if(name == "Lyapunov.Kt") gains.Kt = val;
    else if(name == "mpc.horizon") gains.horizon = val;
    else if(name == "mpc.dt") gains.dt = val;
  }
  return SetParametersResult().set__successful(true);
}

ControlType Robot::declareParam(std::string name,
                                std::map<ControlType, std::string> values)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;

  std::stringstream ss;
  ss << "Control type [";
  const auto total{values.size()};
  bool first{true};
  for(const auto &[val, des]: values)
  {
    if(!first)
      ss << ", ";
    ss << des << " = " << static_cast<int>(val);
    first = false;
  }
  descriptor.set__name(name).set__description(ss.str());
  descriptor.integer_range = {rcl_interfaces::msg::IntegerRange()
                              .set__from_value(0)
                              .set__to_value(total-1)};
  return static_cast<ControlType>(node->declare_parameter<int>(name, 0, descriptor));
}

double Robot::declareParam(std::string name,
                           double default_value,
                           double lower, double upper)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  const auto step{(upper-lower)/1000};
  descriptor.floating_point_range = {rcl_interfaces::msg::FloatingPointRange()
                                     .set__from_value(lower)
                                     .set__to_value(upper)
                                     .set__step(step)};
  default_value = lower + step*std::round((default_value-lower)/step);
  return node->declare_parameter<double>(name, default_value, descriptor);
}

std::array<double, 5> Robot::LyapunovError(const Vector2d &p, const Vector2d &v, const Vector2d &a,
                                           double L, double beta) const
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

void Robot::update(const rclcpp::Time &now)
{
  const auto goal{currentGoal(now)};

  if(gains.control == ControlType::LYAPUNOV)
    computeLyapunovFB(goal);
  else if(gains.control == ControlType::DYNAMIC_FB)
    computeDynamicFB(goal);
  else if(gains.control == ControlType::MPC)
    computeMPC(goal, now);
  else
    computeStaticFB(goal);

  buildCmd(now);
  publish(goal.pd, now);
}

void Robot::publish(const Eigen::Vector2d &pd, const rclcpp::Time &now)
{
  // error
  /* const auto error{pd - xy};
    Float64MultiArray error_msg;
    error_msg.data = {error(0), error(1)};
    error_pub.publish(error_msg);

    Float64 norm_msg;
    norm_msg.data = error.norm();
    norm_pub.publish(norm_msg);*/

  cmd_vel_pub->publish(cmd_vel);

  // carrot
  carrot.header.stamp = now;
  if(gains.control != ControlType::STATIC_FB)
  {
    carrot.position[1] = 99;
  }
  else
  {
    carrot.position[0] = gains.d;
    carrot.position[1] = 0;
  }
  carrot_pub->publish(carrot);
}

std::string Robot::name(rclcpp::Node::SharedPtr node)
{
  const auto rsp_param{std::make_shared<rclcpp::SyncParametersClient>
                      (node, "/robot/robot_state_publisher")};
  std::cout << "Reading robot description... " << std::flush;
  while(true)
  {
    rclcpp::spin_some(node);
    rsp_param->wait_for_service();
    const auto urdf_xml{rsp_param->get_parameter<std::string>("robot_description")};
    if(urdf_xml.empty())
      continue;

    auto model{std::make_unique<urdf::Model>()};
    model->initString(urdf_xml);
    std::cout << " found " << model->getName() << std::endl;
    return model->getName();
  }
/*
  system("ros2 run map_simulator spawn --ros-args -r __ns:=/robot -p
         << " _x:=" << x
         << " _y:=" << y
         << " _theta:=" << theta
         << " _static_tf_odom:=true"
         << " _force_scanner:=false";
      system(ss.str().c_str());)
*/

}
