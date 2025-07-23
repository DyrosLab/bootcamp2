#include <bootcamp2_cc/cc.h>
#include <unistd.h>
#include <chrono>
#include <sstream>

using std::placeholders::_1;

static void pin_compute_fk_jacobian_at_point(
    const pinocchio::Model &model,
    pinocchio::Data &data,
    const Eigen::VectorXd &q,
    pinocchio::FrameIndex fid,
    const Eigen::Vector3d &local_point,
    Eigen::Vector3d &ee_pos_world,
    Eigen::Matrix<double,6,Eigen::Dynamic> &J6_point_out,
    pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::WORLD)
{
  // FK
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  // 프레임 원점 Jacobian
  Eigen::Matrix<double,6,Eigen::Dynamic> J6(6, model.nv);
  pinocchio::computeFrameJacobian(model, data, q, fid, rf, J6);

  // 프레임 SE3
  const pinocchio::SE3 &oMf = data.oMf[fid];

  // 로컬 포인트 → 월드
  Eigen::Vector3d r_world = oMf.rotation() * local_point;
  ee_pos_world = oMf.translation() + r_world;

  // 포인트 오프셋 적용 (linear + w×r)
  J6_point_out.resize(6, model.nv);
  J6_point_out.topRows<3>() = J6.topRows<3>();  // angular
  for (int k = 0; k < model.nv; ++k)
  {
    Eigen::Vector3d w = J6.block<3,1>(0,k);
    Eigen::Vector3d v = J6.block<3,1>(3,k);
    J6_point_out.block<3,1>(3,k) = v + w.cross(r_world);
  }
}


class DyrosSimpleManipulator : public rclcpp::Node
{
public:
  DyrosSimpleManipulator() : Node("Dyros_Simple")
  {
    jointReal_set = this->create_publisher<sensor_msgs::msg::JointState>("motor/joint_states", 5);
    joint_set     = this->create_publisher<sensor_msgs::msg::JointState>("joint_set", 5);

    commanding_sub = this->create_subscription<std_msgs::msg::String>(
      "commanding", 10, std::bind(&DyrosSimpleManipulator::commandingCallback, this, _1));
    time_sub = this->create_subscription<std_msgs::msg::Float32>(
      "exec_time", 10, std::bind(&DyrosSimpleManipulator::timeCallback, this, _1));
    param_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "guiparam", 10, std::bind(&DyrosSimpleManipulator::paramCallback, this, _1));
    joint_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&DyrosSimpleManipulator::jointCallback, this, _1));

    this->declare_parameter<bool>("real", false);
    this->get_parameter("real", real_robot);
  }

  ~DyrosSimpleManipulator() override
  {
    RCLCPP_INFO(this->get_logger(), "Torque Mode off!");
    motorTorqueOff(portHandler, packetHandler);
    RCLCPP_INFO(this->get_logger(), "close the port!");
    portHandler->closePort();
  }

  void commandingCallback(const std_msgs::msg::String msg)
  {
    std::cout << "commandingCallback" << std::endl;
    if (msg.data == "initpose")
    {
      std::cout << "init Pose" << std::endl;
      init_pos = true;
      control_pos = false;
    }
    else if (msg.data == "em")
    {
      torque_off = true;
    }
    else if (msg.data == "start")
    {
      init_pos = false;
      control_pos = true;
    }
  }

  void timeCallback(const std_msgs::msg::Float32 msg)
  {
    trajectory_time = msg.data;
  }

  void paramCallback(const std_msgs::msg::Float32MultiArray msg)
  {
    x_desired.setZero();
    if (msg.data.size() >= 3) {
      x_desired(0) = msg.data[0];
      x_desired(1) = msg.data[1];
      x_desired(2) = msg.data[2];
    }
  }

  void jointCallback(const sensor_msgs::msg::JointState msg)
  {
    std::array<std::string, 6> joint_names{"Rev1", "Rev2", "Rev3", "Rev4", "Rev5", "Rev6"};
    std::lock_guard<std::mutex> lg(global_mutex);
    sw_on = true;
    if(!real_robot)
    {
      for (size_t i = 0; i < msg.name.size(); ++i)
      {
        for (int j = 0; j < 6; ++j)
        {
          if (msg.name[i] == joint_names[j])
          {
            Q_[j] = msg.position[i];
          }
        }
      }
    }
  }

  dynamixel::PortHandler *portHandler = nullptr;
  dynamixel::PacketHandler *packetHandler = nullptr;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr         commanding_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr        time_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr param_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  joint_sub;
};


void model()
{
  rclcpp::Rate loop_rate(Hz);

  std::string urdf_path = "/home/dyros/2025/20250723/ros2_ws/src/bootcamp2_urdf_description/urdf/bootcamp2_urdf.urdf";
  pinocchio::Model model;
  try {
    pinocchio::urdf::buildModel(urdf_path, model);
  } catch (const std::exception &e) {
    std::cerr << "URDF load failed: " << e.what() << std::endl;
    return;
  }
  pinocchio::Data data(model);

  const std::string ee_name = "L6_1";
  pinocchio::FrameIndex fid = model.getFrameId(ee_name);
  if (fid >= model.nframes) {
    std::cerr << "EE frame '" << ee_name << "' not found." << std::endl;
    return;
  }

  Eigen::Vector3d pos;
  pos << -0.05, 0.0, -0.3295;

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);

  Eigen::Matrix<double,6,Eigen::Dynamic> J6_point;  // 6 x nv
  Eigen::Vector3d ee_pos;

  joint_setmsg.position.resize(6);
  joint_setmsg.velocity.resize(6);
  joint_setmsg.effort.resize(6);
  joint_setmsg.name = {"Rev1","Rev2","Rev3","Rev4","Rev5","Rev6"};

  sleep(1);

  while (rclcpp::ok())
  {
    if (init_pos == true && control_pos == false)
    {
      if(tick == 0)
      {
        Q_init = Q_;
      }

      for (int i = 0; i < 6; i++)
      {
        Q_desired(i) = cubic(tick, 0, 2 * Hz, Q_init(i), 30 * DEG2RAD, 0, 0);
      }

      if(tick == 0)
      {
        std::cout << "Go Init pose"<< std::endl;
      }

      {
        std::lock_guard<std::mutex> lg(global_mutex);
        pulse_desired = Q_desired / (DEG2RAD * pulse2deg);  // rad→deg→tick
        control_on = true;
      }

      tick++;

      if (tick == 4 * Hz)
      {
        tick = 0;
        init_pos = false;
        Q_ = Q_desired; // Q update
        std::cout << "finish Init pose"<< std::endl;
      }

      auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
      rclcpp::Time now = clock->now();
      joint_setmsg.header.stamp = now;
      for (int i = 0; i < 6; i++)
        joint_setmsg.position[i] = Q_desired(i);
      joint_set->publish(joint_setmsg);
    }
    else if (init_pos == false && control_pos == true)
    {
      if (tick == 0)
      {
        if (real_robot == true)
        {
          for (int i = 0; i < 6; i++)
            Q_(i) = Q_(i);
        }

        if (model.nq != 6) {
          std::cerr << "WARNING: model.nq != 6; mapping required!\n";
        }
        q = Q_; // assumes nq==6

        // Jacobian & EE pos
        pin_compute_fk_jacobian_at_point(model, data, q, fid, pos, ee_pos, J6_point);

        Eigen::Matrix<double,6,6> J_;
        if (model.nv >= 6)
        {
          J_.block<3,6>(0,0) = J6_point.block<3,6>(3,0); // linear
          J_.block<3,6>(3,0) = J6_point.block<3,6>(0,0); // angular
        }

        end_eff.segment<3>(0) = ee_pos;
        Q_init = Q_;
        Q_desired = Q_;
        tick_init = tick;
        x_init.segment<3>(0) = end_eff.segment<3>(0);
        x_dot.setZero();

        auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
        rclcpp::Time now = clock->now();
        joint_setmsg.header.stamp = now;
        for (int i = 0; i < 6; i++)
          joint_setmsg.position[i] = Q_desired(i);
        joint_set->publish(joint_setmsg);

        std::cout << "Go desired pose"<< std::endl;
      }
      else
      {
        q = Q_desired;
        pin_compute_fk_jacobian_at_point(model, data, q, fid, pos, ee_pos, J6_point);

        Eigen::Matrix<double,6,6> J_;
        if (model.nv >= 6)
        {
          J_.block<3,6>(0,0) = J6_point.block<3,6>(3,0); // linear
          J_.block<3,6>(3,0) = J6_point.block<3,6>(0,0); // angular
        }
        end_eff.segment<3>(0) = ee_pos;
      }

      int final_time = static_cast<int>(trajectory_time * Hz);

      if (tick < tick_init + final_time)
      {
        x_dot(0) = cubicDot(tick, 0, final_time, x_init(0), x_init(0) + x_desired(0), 0, 0);
        x_dot(1) = cubicDot(tick, 0, final_time, x_init(1), x_init(1) + x_desired(1), 0, 0);
        x_dot(2) = cubicDot(tick, 0, final_time, x_init(2), x_init(2) + x_desired(2), 0, 0);
      }
      else
      {
        x_dot.setZero();
      }

      Eigen::Matrix<double,6,6> J_for_inv;
      {
        J_for_inv.setZero();
        if (model.nv >= 6)
        {
          J_for_inv.block<3,6>(0,0) = J6_point.block<3,6>(3,0);
          J_for_inv.block<3,6>(3,0) = J6_point.block<3,6>(0,0);
        }
      }
      Eigen::Matrix<double,6,1> x_dot6 = Eigen::Matrix<double,6,1>::Zero();
      x_dot6.segment<3>(0) = x_dot.segment<3>(0);
      Q_dot = J_for_inv.inverse() * x_dot6;

      Q_desired = Q_desired + Q_dot;

      {
        std::lock_guard<std::mutex> lg(global_mutex);
        pulse_desired = Q_desired / (DEG2RAD * pulse2deg);
        control_on = true;
      }

      tick++;

      if(tick == tick_init + final_time)
      {
        tick = 0;
        init_pos = false;
        control_pos = false;
        std::cout << "finish desired pose"<< std::endl;
      }

      auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
      rclcpp::Time now = clock->now();
      joint_setmsg.header.stamp = now;
      for (int i = 0; i < 6; i++)
        joint_setmsg.position[i] = Q_desired(i);
      joint_set->publish(joint_setmsg);
    }
    else
    {
      // idle
      if(real_robot == true)
      {
        auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
        rclcpp::Time now = clock->now();
        joint_setmsg.header.stamp = now;
        for (int i = 0; i < 6; i++)
          joint_setmsg.position[i] = Q_(i);
        joint_set->publish(joint_setmsg);
      }
    }

    loop_rate.sleep();
  }
}

void motor()
{
  rclcpp::Rate loop_rate(Hz);

  // Dynamixel
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
  dynamixel::GroupSyncRead  groupSyncRead (portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

  for(int i = 0; i < Q_desired.size(); i++)
    jointmsg.position.push_back(0.0);

  if(!serialPortCheck(portHandler)) return;
  motorIDSet();
  motorTorqueOn(portHandler, packetHandler);
  if(!storagePosition(&groupSyncRead)) return;

  while (rclcpp::ok())
  {
    readPosition(&groupSyncRead, packetHandler);
    if(!checkData(&groupSyncRead)) return;
    getPosition(&groupSyncRead);

    {
      std::lock_guard<std::mutex> lg(global_mutex);
      sw_on = true;
      for (int i = 0; i < (int)motorID.size(); i++)
        Q_(i) = present_postiion[i];
    }

    for(int i = 0; i < (int)motorID.size(); i++)
      jointmsg.position[i] = Q_(i);

    jointmsg.name = {"Rev1", "Rev2", "Rev3", "Rev4", "Rev5", "Rev6"};
    jointReal_set->publish(jointmsg);

    if (control_on == true)
    {
      setPosition(&groupSyncWrite, packetHandler, pulse_desired);
      groupSyncWrite.clearParam();
    }

    if (torque_off == true)
    {
      motorTorqueOff(portHandler, packetHandler);
      torque_off = false;
    }

    loop_rate.sleep();
  }

  motorTorqueOff(portHandler, packetHandler);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DyrosSimpleManipulator>();

  std::thread t1(model);
  std::thread t2(motor);

  rclcpp::spin(node);

  t2.join();
  t1.join();

  rclcpp::shutdown();
  return 0;
}
