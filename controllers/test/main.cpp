#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace webots;

int main() {
  cv::Mat img(480, 640, CV_8UC3);
  Robot *robot = new Robot();
  Camera *camera;
  camera = robot->getCamera("camera_sensor");
  //camera->enable()
  //camera->getImage();
    camera->enable(13);

  while(robot->step(32) != -1){
    //mTimeStep = (int)robot->getBasicTimeStep();
    //camera->enable(mTimeStep * 13);
    const unsigned char *image = camera->getImage();
    for(int x =0; x < 640; x++){
      for(int y = 0;y <480;y++){
        int r = camera->imageGetRed(image, 640, x, y);
        int g = camera->imageGetGreen(image, 640, x, y);
        int b = camera->imageGetBlue(image, 640, x, y);
        img.at<cv::Vec3b>(y, x)[0] = b;
        img.at<cv::Vec3b>(y, x)[1] = g;
        img.at<cv::Vec3b>(y, x)[2] = r;
      }
    }

    //std::cout << "hello world" << std::endl;
    cv::imshow("test", img);
    cv::waitKey(1);
  }

    delete robot;
    delete camera;

    return 0;
}
