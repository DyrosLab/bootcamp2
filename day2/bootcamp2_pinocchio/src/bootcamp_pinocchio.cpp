#include <rclcpp/rclcpp.hpp>
#include <string>
#include <iostream>
#include <vector>
#include <cmath>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

// Pinocchio
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

void compute_once(
    const Eigen::VectorXd &q,
    const pinocchio::Model &model,
    pinocchio::Data &data,
    pinocchio::FrameIndex fid,
    const Eigen::Vector3d &pos)
{
    Eigen::Matrix<double,6,Eigen::Dynamic> J6(6, model.nv);
    Eigen::Vector3d end_eff = Eigen::Vector3d::Zero();

    // 1) joint placements
    pinocchio::forwardKinematics(model, data, q);
    // 2) frame placements
    pinocchio::updateFramePlacements(model, data);
    // 3) 프레임 원점에서의 6D Jacobian (WORLD 기준)
    pinocchio::computeFrameJacobian(model, data, q, fid, pinocchio::ReferenceFrame::WORLD, J6);

    // 4) 프레임 placement (W_T_F)
    const pinocchio::SE3 &oMf = data.oMf[fid];

    // 5) 프레임 내 포인트 pos → 월드 좌표
    Eigen::Vector3d r_world = oMf.rotation() * pos;
    end_eff = oMf.translation() + r_world;

    // 6) 포인트 오프셋 적용된 Jacobian
    Eigen::Matrix<double,6,Eigen::Dynamic> J6_point = J6;
    for (int k = 0; k < model.nv; ++k) {
        Eigen::Vector3d w = J6.block<3,1>(0,k); // angular
        Eigen::Vector3d v = J6.block<3,1>(3,k); // linear at frame origin
        J6_point.block<3,1>(3,k) = v + w.cross(r_world);
    }

    // 출력
    std::cout << "\n=== Jacobian 6D @ point (angular; linear) ===" << std::endl;
    std::cout << J6_point << std::endl;

    std::cout << "end_effector position (world):\n";
    std::cout << "  x: " << end_eff.x() << "\n";
    std::cout << "  y: " << end_eff.y() << "\n";
    std::cout << "  z: " << end_eff.z() << std::endl;
}

int main(int argc, char* argv[])
{
  const std::string node_name = "bootcamp_pinocchio";
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(node_name);
  rclcpp::Rate loop_rate(1);  // 1 Hz

  std::string urdf_path = "/home/dyros/2025/20250723/ros2_ws/src/bootcamp2_urdf_description/urdf/bootcamp2_urdf.urdf";
  if (argc > 1) {
    urdf_path = argv[1];
  }
  RCLCPP_INFO(node->get_logger(), "Loading URDF: %s", urdf_path.c_str());

  pinocchio::Model model;
  try {
    pinocchio::urdf::buildModel(urdf_path, model);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "URDF load failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  pinocchio::Data data(model);

  RCLCPP_INFO(node->get_logger(), "Model loaded. nq=%d nv=%d njoints=%d nframes=%d",
              model.nq, model.nv, model.njoints, model.nframes);

  const std::string ee_name = "L6_1";
  pinocchio::FrameIndex fid = model.getFrameId(ee_name);
  if (fid == (pinocchio::FrameIndex)(-1) || fid >= model.nframes) {
    RCLCPP_ERROR(node->get_logger(), "Frame '%s' not found in model.", ee_name.c_str());
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Using frame '%s' (id=%u).", ee_name.c_str(), (unsigned)fid);

  Eigen::Vector3d pos;
  pos << -0.05, 0.0, -0.3295;

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  q.setRandom();
  q *= 0.5;

  // 초기 1회 계산
  compute_once(q, model, data, fid, pos);

  while (rclcpp::ok())
  {
    q.setRandom();
    q *= 0.5;
    compute_once(q, model, data, fid, pos);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}