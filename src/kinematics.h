#pragma once

#include <BasicLinearAlgebra.h>
#include "utils.h"

struct KinematicsConfig
{
  float l1;
  float l2;
  float l3;
};

// returns a rotation matrix about x according to angle theta
BLA::Matrix<3,3> rot_mat_x(double theta) {
  BLA::Matrix<3, 3> rot_x = {1,
                          0,
                          0,
                          0,
                          cos(theta),
                          -sin(theta),
                          0,
                          sin(theta),
                          cos(theta)};
  return rot_x;
}

// returns a rotation matrix about y according to angle theta
BLA::Matrix<3,3> rot_mat_y(double theta) {
  BLA::Matrix<3, 3> rot_y = {cos(theta),
                            0,
                            sin(theta),
                            0,
                            1,
                            0,
                            -sin(theta),
                            0,
                            cos(theta)};

  return rot_y;
}

// returns a rotation matrix about z according to angle theta
BLA::Matrix<3,3> rot_mat_z(float theta) {
  BLA::Matrix<3, 3> rot_z = {cos(theta),
                            -sin(theta),
                            0,
                            sin(theta),
                            cos(theta),
                            0,
                            0,
                            0,
                            1};
  return rot_z;
}

// TODO: Step 12. Implement forward kinematics
BLA::Matrix<3> forward_kinematics(const BLA::Matrix<3> &joint_angles, const KinematicsConfig &config)
{
  // /* Computes forward kinematics for the 3DOF robot arm/leg.
  
  // Returns the cartesian coordinates of the end-effector 
  // corresponding to the given joint angles and leg configuration. 
  // */
}

BLA::Matrix<3> inverse_kinematics(const BLA::Matrix<3> &target_location, const KinematicsConfig &config)
{
  return BLA::Matrix<3>(0, 0, 0);
}

enum class BodySide
{
  kLeft,
  kRight,
  kUnspecified
};

BLA::Matrix<3> correct_for_actuator_direction(const BLA::Matrix<3> &joint_vector, const BodySide &side)
{
  return {joint_vector(0), joint_vector(1), joint_vector(2)};
}
