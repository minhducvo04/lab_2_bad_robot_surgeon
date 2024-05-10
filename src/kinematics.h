#pragma once
#include <BasicLinearAlgebra.h>
#include "utils.h"
struct KinematicsConfig
{
  float l[3];
};

BLA::Matrix<3> translation(const float value, const char type){
  // Purpose: converting a scalar into a vector
  // Input: value: L1, L2, or L3, type: x, y, z (axis where it is initialize: z is upright, y is in direction of L1, the other axis is x)
  // Output: return [0, L1, 0], [0, 0, L2], or [0, 0, L3] depending on how you call the function
  if(type == 'x'){
    return {value, 0, 0};
  }
  if(type == 'y'){
    return {0, value, 0};
  }
  return {0, 0, value};
}

BLA::Matrix<3> rotation(const BLA::Matrix<3> m, const float theta, const char type){
  // Finding the final Cartesian coordinate of T after rotating by theta in a plane
  // Input: theta: angle rotating, type: normal vector of your plane of rotating (example: L1 rotating in xy -> type = 'z')
  // Output: Cartesian coordinate
  BLA::Matrix<3, 3> rot;
  if(type == 'x'){
    rot = {1, 0, 0,
            0, cos(theta), -sin(theta),
            0, sin(theta), cos(theta)};
  }
  else{
    if(type == 'y'){
      rot =  {cos(theta), 0, -sin(theta),
                      0, 1, 0,
                      sin(theta), 0, cos(theta)};
    }
    else{
      rot = {cos(theta), -sin(theta), 0,
            sin(theta), cos(theta), 0,
            0, 0, 1};   
    }
  } 
  BLA::Matrix<3> ret = rot * m;
  return ret;                      
}

// TODO: Step 12. Implement forward kinematics
//, float x, float y, float z)
BLA::Matrix<3> forward_kinematics(const BLA::Matrix<3> &joint_angles, const KinematicsConfig &config)\
{
  // Purpose: Finding the final Cartesian coordinate of the system of 3L after each rotate by some angle 'joint_angles'
  // Input: joint_angles: angles rotating by Ls, config: length of the Ls
  // Output: Cartesian coordinate
  /* Computes forward kinematics for the 3DOF robot arm/leg.
  
  Returns the cartesian coordinates of the end-effector corresponding to the given joint angles and leg configuration. 
  */
  // float x = 0;
  // float y = 0;
  // float z = 0;
  BLA::Matrix<3> T = translation(config.l[2], 'z');
  for(int i = sizeof(config) - 2; i >= 0; i--){
    // BLA::Matrix <3> trans = translation(config.l[i], 'z');
    // BLA::Matrix <3,3> rot = rotation(joint_angles(i), 'y');
    T = rotation(T, joint_angles(i + 1), 'y');
    if(i){
      T = translation(config.l[i], 'z') + T;
    }
    else{
      T = translation(config.l[i], 'y') + T;
    }
  }
  T = rotation(T, joint_angles(0), 'z');
  // BLA::Matrix<3> temp = {0, config.l[sizeof(config) - 1], 1};
  // BLA::Matrix<3> pos = T * temp;
  return T;
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
