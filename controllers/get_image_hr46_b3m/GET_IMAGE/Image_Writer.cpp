#include "Image_Writer.hpp"

namespace fs = std::filesystem;


Image_Writer::Image_Writer() : enable_capture(false), image_count(0)
{
  createCapturedImageDirectory();
  enable_capture = true;
  last_capture_time = std::chrono::high_resolution_clock::now();
  /*const char *env_capture = std::getenv("CAPTURE");
  if(env_capture && env_capture[0] == '1') {
    createCapturedImageDirectory();
    enable_capture = true;
    last_capture_time = std::chrono::high_resolution_clock::now();
  }
  */
}

Image_Writer::~Image_Writer()
{

}

void Image_Writer::createCapturedImageDirectory(void)
{
  std::string capture_dir("images");
  if(!fs::exists(capture_dir)) {
    fs::create_directory(capture_dir);
  }
  
  std::stringstream ss_capture;
  auto now_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  ss_capture << capture_dir << "/" << std::put_time(std::localtime(&now_t), "%Y%m%dT%H%M%S");
  capture_save_path = ss_capture.str();
  fs::create_directory(capture_save_path);
}

std::string Image_Writer::get_capture_save_path(void)
{
  return capture_save_path;
}

std::string Image_Writer::save_captured_image(std::string filename)
{
  std::stringstream ss;
  ss << filename << "/" << std::setw(6) << std::setfill('0') << image_count << ".jpg";
  image_count++;
  std::string save_image_name(ss.str());
  return save_image_name;
}

std::string Image_Writer::save_captured_image(void)
{
  std::stringstream ss;
  ss << capture_save_path << "/" << std::setw(6) << std::setfill('0') << image_count << ".jpg";
  image_count++;
  std::string save_image_name(ss.str());
  return save_image_name;
}
