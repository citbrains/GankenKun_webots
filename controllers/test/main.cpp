#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <string>
//#include <experimental/filesystem>
#include <filesystem>
#include <iomanip>

//namespace fs = std::experimental::filesystem;
namespace fs = std::filesystem;
using namespace webots;

int main() {
  cv::Mat img(480, 640, CV_8UC3);
  Robot *robot = new Robot();
  Camera *camera;
  camera = robot->getCamera("camera_sensor");
  //camera->enable()
  //camera->getImage();
  camera->enable(13);

  time_t t = time(nullptr);
  const tm* localTime = localtime(&t);
  std::stringstream s;
  // s << localTime->tm_year + 1900;
  // setw(),setfill()で0詰め
  s << std::setw(2) << std::setfill('0') << localTime->tm_mon + 1;
  s << std::setw(2) << std::setfill('0') << localTime->tm_mday;
  s << std::setw(2) << std::setfill('0') << localTime->tm_hour;
  s << std::setw(2) << std::setfill('0') << localTime->tm_min;
  // s << std::setw(2) << std::setfill('0') << localTime->tm_sec;
  // fs::create_directory(s.str());

  while(robot->step(32) != -1){
    static int i = 0;
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
    // cv::imshow("test", img);
    std::ostringstream oss;
    oss << std::setfill( '0' ) << std::setw( 4 ) << i++;
    cv::imwrite( "/home/kanbe/GankenKun_webots/controllers/test/images/image_" + oss.str() + ".png", img );
    // cv::imwrite( s.str() + "/" + "output_" + oss.str() + ".jpg", img );

    cv::imshow( "image", img );
    cv::waitKey(1);
    // imagewriter.saveCapturedImage(img);
  }

    delete robot;
    delete camera;

    return 0;
}
