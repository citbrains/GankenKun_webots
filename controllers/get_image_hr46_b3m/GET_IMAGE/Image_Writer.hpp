#ifndef _IMAGE_WRITER_HPP_
#define _IMAGE_WRITER_HPP_

#include <iostream>
#include <vector>
#include <chrono>
#include <sstream>
#include <cstdlib>
#include <iomanip>
#include <filesystem>


class Image_Writer{
public:
  Image_Writer();
  ~Image_Writer();
  void createCapturedImageDirectory(void);
  std::string get_capture_save_path(void);
  std::string save_captured_image(std::string );
  std::string save_captured_image(void);
private:
  bool enable_capture;
  int image_count;
  std::chrono::time_point<std::chrono::high_resolution_clock> last_capture_time;
  std::string capture_save_path;
};
#endif //_IMAGE_WRITER_HPP_ 
