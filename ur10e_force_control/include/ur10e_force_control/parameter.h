#pragma once
#include <modern_robotics.h>



Eigen::Vector3d gravity;

Eigen::Matrix4d m01;
Eigen::Matrix4d m12;
Eigen::Matrix4d m23;
Eigen::Matrix4d m34;
Eigen::Matrix4d m45;
Eigen::Matrix4d m56;
Eigen::Matrix4d m67;

Eigen::MatrixXd Slist(6,6);

Eigen::MatrixXd G1;
Eigen::MatrixXd G2;
Eigen::MatrixXd G3;
Eigen::MatrixXd G4;
Eigen::MatrixXd G5;
Eigen::MatrixXd G6;

std::vector<Eigen::MatrixXd> Mlist;
std::vector<Eigen::MatrixXd> Glist;
int hz;
void setup();

Eigen::MatrixXd compute_joint_torque(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist, const Eigen::VectorXd& thetaend, const double Tf, const int method);