// File:          GankenKun_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <GankenKun_controller.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

static const char *motorNames[NMOTORS] = {
  "head_yaw_joint" /*ID1 */,
  "left_shoulder_pitch_joint [shoulder]" /*ID2 */,
  "left_shoulder_roll_joint" /*ID3 */,
  "left_elbow_pitch_joint" /*ID4 */,
  "right_shoulder_pitch_joint [shoulder]" /*ID5 */,
  "right_shoulder_roll_joint" /*ID6 */,
  "right_elbow_pitch_joint" /*ID7 */,
  "left_waist_yaw_joint" /*ID8 */,
  "left_waist_roll_joint [hip]" /*ID9 */,
  "left_waist_pitch_joint" /*ID10*/,
  "left_knee_pitch_joint" /*ID11*/,
  "left_ankle_pitch_joint" /*ID12*/,
  "left_ankle_roll_joint" /*ID13*/,
  "right_waist_yaw_joint" /*ID14*/,
  "right_waist_roll_joint [hip]" /*ID15*/,
  "right_waist_pitch_joint" /*ID16*/,
  "right_knee_pitch_joint" /*ID17*/,
  "right_ankle_pitch_joint" /*ID18*/,
  "right_ankle_roll_joint" /*ID19*/,
};

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  double minMotorPositions[NMOTORS];
  double maxMotorPositions[NMOTORS];
  Motor *mMotors[NMOTORS];
  PositionSensor *mPositionSensors[NMOTORS];
  Robot *robot = new Robot();

  // get the time step of the current world.
  int mTimeStep = (int)robot->getBasicTimeStep();

  for (int i = 0; i < NMOTORS; i++) {
    mMotors[i] = robot->getMotor(motorNames[i]);
    string sensorName = motorNames[i];
    sensorName += "_sensor";
    mPositionSensors[i] = robot->getPositionSensor(sensorName);
    mPositionSensors[i]->enable(mTimeStep);
    minMotorPositions[i] = mMotors[i]->getMinPosition();
    maxMotorPositions[i] = mMotors[i]->getMaxPosition();
  }

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  double angle = 0.0, delta_angle = 0.01;
  const double lower_angle = 0.0, upper_angle = 1.57;
  while (robot->step(mTimeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
    angle += delta_angle;
    if (angle >= upper_angle) delta_angle *= -1.0;
    if (angle <= lower_angle) delta_angle *= -1.0;
    mMotors[2]->setPosition(0.75);
    mMotors[3]->setPosition(-1.0);
    mMotors[5]->setPosition(-0.75);
    mMotors[6]->setPosition(-1.0);
    mMotors[9]->setPosition( angle);
    mMotors[10]->setPosition(-angle);
    mMotors[15]->setPosition( angle);
    mMotors[16]->setPosition(-angle);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}

