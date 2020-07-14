#include <iostream>
#include <yaml-cpp/yaml.h>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>

#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/addons/urdfreader/urdfreader.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main (int argc, char* argv[]) {
  YAML::Node config = YAML::LoadFile("/home/lee/unhuman/realtime-rbdl/src/param.yaml");
  rbdl_check_api_version (RBDL_API_VERSION);

  Model model;
  auto model_urdf_file = config["model"].as<std::string>();

  if (!Addons::URDFReadFromFile (model_urdf_file.c_str(), &model, false)) {
    std::cerr << "Error loading model " << model_urdf_file << std::endl;
    abort();
  }

  auto tmp = config["joint_to_actuator"].as<std::vector<std::vector<double>>>();
  Eigen::MatrixXd joint_to_actuator(tmp.size(),tmp[0].size());
  for (int i=0; i<tmp.size(); i++) {
      for (int j=0; j<tmp[0].size(); j++) {
        joint_to_actuator(i,j) = tmp[i][j];
      }
  }
  std::cout << joint_to_actuator << std::endl;

  std::cout << "Degree of freedom overview:" << std::endl;
  std::cout << Utils::GetModelDOFOverview(model);

  std::cout << "Model Hierarchy:" << std::endl;
  std::cout << Utils::GetModelHierarchy(model);

  std::cout << Utils::GetNamedBodyOriginsOverview(model);

  VectorNd Q = VectorNd::Zero (model.q_size);
  VectorNd QDot = VectorNd::Zero (model.qdot_size);
  VectorNd Tau = VectorNd::Zero (model.qdot_size);
  VectorNd QDDot = VectorNd::Zero (model.qdot_size);

  std::cout << "Forward Dynamics with q, qdot, tau set to zero:" << std::endl;
  for (int i=0;i<10000;i++)
      ForwardDynamics (model, Q, QDot, Tau, QDDot);
  std::cout << QDDot.transpose() << std::endl;

  Eigen::MatrixXd J(6,model.mJoints.size()-1); 
  J.setZero();
  CalcBodySpatialJacobian(model, Q, model.mBodyNameMap["RFinger"], J);
  std::cout << model.gravity.transpose() << std::endl;

  // impedance control with tension
  // theta_j = joint_to_actuator * theta_m
  // cartesian measured -> xmeas = kinematics(theta_j)
  // Cartesian input -> Fdes = kp*(xdes - xmeas) + Fdes_ff
  // torque desired -> tau_des = J'*Fdes;
  // actuator torque = joint_to_actuator' * tau_des 
  // actuator_torque += -min(actuator_torque) + min_tension

  // position control with force and tension
  // 
  // tension_offset = PI(tension_desired - tension_measured)
  // actuator_position = joint_to_actuator*joint_des
  // actuator_position += tension_offset

  std::cout << J << std::endl;


  return 0;
}
