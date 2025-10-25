/**
 * @file OV9715.cpp
 * @brief Class for stereo camera
 * @details Run info:
 *            sudo apt install libopencv-dev
 *            g++ OV9715.cpp -o OV9715 `pkg-config --cflags --libs opencv4`
 *            ./OV9715
 */

#include <opencv2/opencv.hpp>
#include <cassert>

class OV9715
{
public:
  OV9715 (int port = 0)
  {
    stereo_cam = cv::VideoCapture (port);
  }

  /**
   * For WSL testing
   */
  OV9715 (std::string source)
  {
    stereo_cam = cv::VideoCapture (source);
  }

  void process_frame (bool debug = false)
  {
    capture_frame (debug);
    build_depth_map (debug);
  }

private:
  cv::VideoCapture stereo_cam;
  cv::Mat raw_frame;
  cv::Mat left_img;
  cv::Mat right_img;
  cv::Mat depth_map;

  void capture_frame (bool debug = false)
  {
    stereo_cam >> raw_frame;

    left_img = raw_frame (cv::Range (0, raw_frame.rows), 
                          cv::Range (0, raw_frame.cols / 2));

    right_img = raw_frame (cv::Range (0, raw_frame.rows), 
                          cv::Range (raw_frame.cols / 2, raw_frame.cols));

    if (debug)
    {
      cv::imwrite ("./out/left.jpg", left_img);
      cv::imwrite ("./out/right.jpg", right_img);  
    }
  }

  void build_depth_map (bool debug = false)
  {
    assert (!left_img.empty ());
    assert (!right_img.empty ());

    // TODO: Rectify?

    // TODO: Compute disparity map

  }
};

int main ()
{
  OV9715 ov9715 = OV9715 ("stereo-small.jpg");
  ov9715.process_frame (debug = true);
}