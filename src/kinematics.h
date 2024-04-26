#pragma once

#include <BasicLinearAlgebra.h>
#include "utils.h"

struct KinematicsConfig
{
  float l1;
  float l2;
  float l3;
};
BLA::Matrix<4,4> rotation(const float theta, const float x, const float y, const float z){
  BLA::Matrix<4,4> translationalMatrix = {1,0,0,x,
                                          0,1,0,y,
                                          0,0,1,z,
                                          0,0,0,1};
  BLA::Matrix<4,4> rotationalMatrix = {cos(PI + theta), -sin(PI + theta), 0, 0,
                                        sin(PI + theta), cos(PI + theta), 0 , 0,
                                        0,0,1,0,
                                        0,0,0,1};

  BLA::Matrix<4,4> matrixT = translationalMatrix * rotationalMatrix;

  return matrixT;
}

// TODO: Step 12. Implement forward kinematics
BLA::Matrix<3> forward_kinematics(const BLA::Matrix<3> &joint_angles, const KinematicsConfig &config)
{
  /* Computes forward kinematics for the 3DOF robot arm/leg.
  
  Returns the cartesian coordinates of the end-effector corresponding to the given joint angles and leg configuration. 
  */

  /* Suggested Implementation
      Refer to Slide 38 in the FK Lecture Slides
      Parameters: Joint angles for each motor, config for the joint offsets
      Create helper functions to perform a rotation and translation together
        Parameters: theta, x, y, z
        Return: 4x4 Matrix for the corresponding translation (homogeneous coordinates)
      Call each transformation helper function together in this FK function, returning the cartesian coordinates in x, y, z
      Return: 3x1 Vector (BLA::Matrix<3,1>) for the x, y, z cartesian coordinates
  */ 
  // T = transla
  // return BLA::Matrix<3>(0, 0, 0);
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
