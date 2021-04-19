// File:          GET_IMAGE.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include "Image_Writer.hpp"

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main() {
  // create the Robot instance.
  Robot *robot = new Robot();
  Camera *camera = robot->getCamera("camera_sensor");
  Image_Writer *image_writer = new Image_Writer();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  camera->enable(timeStep);
  

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  //std::string filename(image_writer->get_capture_save_path());
  //std::cout << filename << std::endl;
  //std::string filename("image");
  //int image_count=0;
  while (robot->step(timeStep) != -1) {
    camera->getImage();
    /*
    std::stringstream ss;
    std::cout << filename << std::endl;
    ss << filename << "/" << std::setw(6) << std::setfill('0') << image_count << ".jpg";
    image_count++;
    std::string save_image_name(ss.str());
    std::cout << save_image_name << std::endl;
    */
    camera->saveImage(image_writer->save_captured_image(), 100);
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.
  delete camera;
  delete robot;
  return 0;
}
