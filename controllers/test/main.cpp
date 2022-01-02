#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <string>
//#include <experimental/filesystem>
#include <filesystem>

//namespace fs = std::experimental::filesystem;
namespace fs = std::filesystem;
using namespace webots;

// class ImageWriter
// {
// public:
// 	ImageWriter();
// 	~ImageWriter();
// 	void createCapturedImageDirectory();
// 	void saveCapturedImage(cv::Mat);
// private:
// 	bool enable_capture;
// 	int image_count;
// 	std::chrono::time_point<std::chrono::high_resolution_clock> last_capture_time;
// 	std::string capture_save_path;
// };
//
// ImageWriter::ImageWriter() : enable_capture(false), image_count(0)
// {
// 	createCapturedImageDirectory();
// 	enable_capture = true;
// 	last_capture_time = std::chrono::high_resolution_clock::now();
// }
//
//
// ImageWriter::~ImageWriter()
// {
// }
//
// void ImageWriter::createCapturedImageDirectory(void)
// {
//   std::string capture_save_path;
//   std::string capture_dir("game_images");
//   if(!fs::exists(capture_dir)) {
//     fs::create_directory(capture_dir);
//   }
//   std::stringstream ss_capture;
//   auto now_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
//   ss_capture << capture_dir << "/" << std::put_time(std::localtime(&now_t), "%Y%m%dT%H%M%S");
//   capture_save_path = ss_capture.str();
//   fs::create_directory(capture_save_path);
// }
//
// void ImageWriter::saveCapturedImage(cv::Mat img)
// {
//   static int image_count = 0;
//   bool enable_capture = true;
//   if(enable_capture) {
//     const auto capture_interval = std::chrono::seconds(1);
//     if(std::chrono::high_resolution_clock::now() - last_capture_time > capture_interval) {
//       std::stringstream ss;
//       ss << capture_save_path << "/" << std::setw(6) << std::setfill('0') << image_count << ".jpg";
//       image_count++;
//       cv::Mat save_img;
//       cv::cvtColor(img, save_img, CV_YCrCb2BGR);
//       cv::imwrite(ss.str().c_str(), save_img);
//       last_capture_time = std::chrono::high_resolution_clock::now();
//     }
//   }
// }

int main() {
  // ImageWriter imagewriter;
  cv::Mat img(480, 640, CV_8UC3);
  Robot *robot = new Robot();
  Camera *camera;
  camera = robot->getCamera("camera_sensor");
  //camera->enable()
  //camera->getImage();
  camera->enable(13);
  // imagewriter.createCapturedImageDirectory();

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
    // imagewriter.saveCapturedImage(img);
  }

    delete robot;
    delete camera;

    return 0;
}
