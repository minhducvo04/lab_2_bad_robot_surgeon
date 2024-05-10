#include "Arduino.h"
#include "C610Bus.h"
#include <array>
#include "BasicLinearAlgebra.h"
#include "pid.h"
#include "kinematics.h"
#include "test_kinematics.h"
#include "utils.h"
#include "bits/stdc++.h"
// Examine your robot leg to see if you built a left leg or a right leg.
// Then replace kUnspecified with the correct side.
const BodySide kLegSide = BodySide::kLeft; // Replace with BodySide::kLeft or BodySide::kRight
/*********************************************************/

// #define DO_TESTS = // comment out to skip running tests before executing main program

long last_command = 0; // To keep track of when we last commanded the motors
C610Bus<CAN1> bus_front;     // Initialize the Teensy's CAN bus to talk to the motors
C610Bus<CAN2> bus_back;

const int LOOP_DELAY_MILLIS = 1; // Wait for 0.005s between motor updates.

const float m1_offset = 0.0;
const float m2_offset = 0.0;

typedef struct {
  float pos;
  float vel;
  float cmd;
} MotorState;

MotorState front_state[3]; // MotorStrate struct array for updating the motor state of the front leg, update these in your code
MotorState back_state[3]; // MotorStrate struct array for updating the motor state of the back leg, update these in your code

BLA::Matrix<3> actuator_angles{0, 0, 0};     // rad
BLA::Matrix<3> actuator_velocities{0, 0, 0}; // rad/s
BLA::Matrix<3> actuator_commands{0, 0, 0};   // mA

const float Kp = 1000;
const float Kd = 500;
const float kMaxCurrent = 2000;
// BLA::Matrix<3> L{3,8.5,11.75};
// Define the signed hip offset and link lengths
const KinematicsConfig pupper_leg_config = (kLegSide == BodySide::kLeft) ? KinematicsConfig{0.035, 0.08, 0.11} : KinematicsConfig{-0.035, 0.08, 0.11};

// updates the given motor position and velocity according to the bus ID
void updateState(MotorState* state, int id, int bus_id) {
  if (bus_id) {
    state->pos = bus_front.Get(id).Position();
    state->vel = bus_front.Get(id).Velocity();
  }
  else {
    state->pos = bus_back.Get(id).Position();
    state->vel = bus_back.Get(id).Velocity();
  }
}

// updates the given motor command current
void updateCmd(MotorState* state, float target, float kp, float kd) {
  state->cmd = pd_control(state->pos, state->vel, target, kp, kd);
}

int start_time;
// This code waits for the user to type s before executing code.
void setup()
{
  // Remove all characters that might have been stored up in the serial input buffer prior to running this program
  while (Serial.available()) {
    Serial.read();
  }
  long last_print = millis();
  while (true)
  {
    char c = Serial.read();
    if (c == 's')
    {
      Serial.println("Starting code.");
      break;
    }
    if (millis() - last_print > 2000) {
      Serial.println("Press s to start, start legs vertically as shown in the lab spec.");
      last_print = millis();
    }
  }
  #ifdef DO_TESTS
    test_kinematics();
  #endif
  start_time = millis();
}

float flip = 1.0;

void control(float target){
  for(int i = 0; i < 3; i++){
      updateState(&back_state[i], i, 0);
    }
    for(int i = 0; i < 3; i++){
      updateCmd(&back_state[i], front_state[i].pos, Kp, Kd);
    }
    for(int i = 0; i < 3; i++){
      updateState(&front_state[i], i, 1);
    }
    for(int i = 0; i < 3; i++){
      updateCmd(&front_state[i], target, Kp, Kd);
    }
    // Sanitizes your computed current commands to make the robot safer. Use this for all parts, leave as is.
    for (int i = 0; i < 3; i++) {
      sanitize_current_command(back_state[i].cmd, back_state[i].pos, back_state[i].vel); // sanitize right leg commands
      sanitize_current_command(front_state[i].cmd, front_state[i].pos, front_state[i].vel); // sanitize left leg commands
    }
    // this block commands the currents on both the front and back buses, leave as is.
    bus_back.CommandTorques(back_state[0].cmd, back_state[1].cmd, back_state[2].cmd, 0 , C610Subbus::kOneToFourBlinks);
    bus_front.CommandTorques(front_state[0].cmd, front_state[1].cmd, front_state[2].cmd, 0 , C610Subbus::kOneToFourBlinks);
}

void loop()
{
  bus_back.PollCAN(); // Check for messages from the motors.
  bus_front.PollCAN();
  long now = millis();

  if (Serial.available())
  {
    if (Serial.read() == 's')
    {
      bus_front.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks);
      bus_back.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks);
      Serial.println("Stopping.");
      while (true)
      {
      }
    }
  }

  if (now - last_command >= LOOP_DELAY_MILLIS)
  {
    // PART ONE: Bad Robot Surgeon
    // TODO: Steps 1.5, 6, 9, 10. update all the motor states according to their ID, indexed at 0
    // HINT: Use the updateState and updateCmd functions similar to lab 1. Remember that there are both a front and a back CAN bus.
    
    // CHECK control()
  
    // Block for printing the motor positions of all the motors, leave as is.
    for (int i = 0; i < 3; i++) {
      if (i == 0) {
        Serial.println("______Motor Positions______");
      }
      Serial.print("Back Leg Motor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(bus_back.Get(i).Position());

      Serial.print("Front Leg Motor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(bus_front.Get(i).Position());
      Serial.println("");
    }

    //PART TWO: Forward Kinematics
    // Uncomment this block when starting the forward kinematics part. This for loop gets the actuator angles for each motor
    for (int i = 0; i < 3; i++)
    {
      actuator_angles(i) = bus_front.Get(i).Position() - 0.08;
      actuator_velocities(i) = bus_front.Get(i).Velocity();
    }
    BLA::Matrix<3> cartesian_coordinates = forward_kinematics(actuator_angles, pupper_leg_config); // This line finds the cartesian coordinates from the forward kinematics function
    print_vector(cartesian_coordinates); // use the print_vector functin to cleanly print out the cartesian coordinates
    // Serial.println(pupper_leg_config.l[0] >= 0.4);
    // print_vector(rotation(translation(pupper_leg_config.l[2], 'z'), actuator_angles(2), 'y'));
    // Serial.println(actuator_angles(0));
    // Serial.println(actuator_angles(1)); 
    // Serial.println(actuator_angles(2));
    // TODO: Step 14. Create a Safety Box
    // Check if the cartesian coordinates are outside a box you determine. If outside, print OUTSIDE SAFETY BOX
    // float a = pupper_leg_config.l[1] + pupper_leg_config.l[2];
    // float b = pupper_leg_config.l[0];
    // BLA::Matrix<3> safety_angle = {0.8, 0.8, 0.8};
    // BLA::Matrix<3> safety_box = forward_kinematics(safety_angle, pupper_leg_config);
    bool unsafe = false;
    // BLA::Matrix<3> max_capacity = {sqrt(a * a + b * b), sqrt(a * a + b * b), b};
    // for(unsigned int i = 0; i < 3; i++){
    if(abs(cartesian_coordinates(0)) > 0.06){
      Serial.println("-----RUNNNNN!!!!! OUTSIDE SAFETY BOX-----");
      unsafe = true;
    }
    // }
    
    float time = millis() / 1000.0; // millis() returns the time in milliseconds since start of program
    float target = sin(time);
    // TODO: Step 15. Do a safety dance if outside the safety bounds
    // In the for loop from Step 14, command one arm to oscillate every cycle to create haptic feedback
    // HINT: use the flip variable to alternate the current one very cycle of the control loop
    Serial.println(unsafe);
    if(unsafe){
      target = 0;
      unsafe = false;
    }
    // else{
    control(target);
    // }
    // Sanitizes the currents for safety, leave as is
    actuator_commands = vectorized_sanitize(actuator_commands,
                                            actuator_angles,
                                            actuator_velocities,
                                            kMaxCurrent);

    last_command = now;
    Serial.println();
  }
}