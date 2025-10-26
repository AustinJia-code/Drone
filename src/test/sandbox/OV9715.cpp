/**
 * @file OV9715.cpp
 * @brief Class for stereo camera
 * @details Run info:
 *            1. sudo apt install libopencv-dev
 *            2. Run calibrate.cpp or upload a calibration yml
 *            3. g++ OV9715.cpp -o OV9715 `pkg-config --cflags --libs opencv4`
 *            4. ./OV9715
 * @cite Useful information from:
 * https://www.cs.cmu.edu/~16385/s17/Slides/13.1_Stereo_Rectification.pdf
 * https://www.cs.cmu.edu/~16385/s17/Slides/13.2_Stereo_Matching.pdf
 * https://learnopencv.com/camera-calibration-using-opencv/
 */

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <cassert>

class OV9715
{
public:
  OV9715 (const std::string& calib_file = "stereo_calib.yml", const std::string& source = "")
  {
    loadCalibration(calib_file);

    if (!source.empty ())
      stereo_cam = cv::VideoCapture (source);
  }

  OV9715(const std::string& calib_file = "stereo_calib.yml", int port = 0)
  {
    loadCalibration (calib_file);
    stereo_cam = cv::VideoCapture(port);
  }

  void process_frame (bool debug = false)
  {
    capture_frame (debug);
    build_depth_map (debug);
  }

private:
  cv::VideoCapture stereo_cam;

  // Stereo calibration matrices
  cv::Mat K1, D1, K2, D2;
  cv::Mat R1, R2, P1, P2, Q;
  cv::Size img_size;

  // Images
  cv::Mat raw_frame, left_img, right_img, depth_map;

  void loadCalibration (const std::string& calib_file)
  {
    cv::FileStorage fs (calib_file, cv::FileStorage::READ);

    fs["K1"] >> K1;
    fs["D1"] >> D1;
    fs["K2"] >> K2;
    fs["D2"] >> D2;
    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["Q"] >> Q;
    fs.release ();
  }

  void split_frame (const cv::Mat& frame, cv::Mat& left, cv::Mat& right)
  {
    left = frame (cv::Range (0, frame.rows),
                  cv::Range (0, frame.cols / 2)).clone ();
    right = frame (cv::Range(0, frame.rows),
                   cv::Range(frame.cols / 2, frame.cols)).clone ();
  }

  void capture_frame (bool debug = false)
  {
    stereo_cam >> raw_frame;

    img_size = cv::Size (raw_frame.cols / 2, raw_frame.rows);

    split_frame (raw_frame, left_img, right_img);

    if (debug)
    {
      cv::imwrite ("./out/raw_left.jpg", left_img);
      cv::imwrite ("./out/raw_right.jpg", right_img);
    }
  }

  void build_depth_map (bool debug = false)
  {
    assert (!left_img.empty () && !right_img.empty ());

    // Undistort and rectify
    cv::Mat map1_left, map2_left, map1_right, map2_right;
    cv::initUndistortRectifyMap (K1, D1, R1, P1, img_size, CV_32FC1,
                                  map1_left, map2_left);
    cv::initUndistortRectifyMap (K2, D2, R2, P2, img_size, CV_32FC1,
                                  map1_right, map2_right);

    cv::Mat left_rect, right_rect;
    cv::remap (left_img, left_rect, map1_left, map2_left, cv::INTER_LINEAR);
    cv::remap (right_img, right_rect, map1_right, map2_right, cv::INTER_LINEAR);

    if (debug)
    {
      cv::imwrite ("./out/rectified_left.jpg", left_rect);
      cv::imwrite ("./out/rectified_right.jpg", right_rect);
    }

    // Convert to grayscale
    cv::Mat left_gray, right_gray;
    cv::cvtColor (left_rect, left_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor (right_rect, right_gray, cv::COLOR_BGR2GRAY);

    // Compute disparity
    int numDisparities = 16 * 6;
    int blockSize = 11;
    cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create (
      0, numDisparities, blockSize,
      8 * 3 * blockSize * blockSize,
      32 * 3 * blockSize * blockSize,
      1, 63, 10, 100, 32,
      cv::StereoSGBM::MODE_SGBM_3WAY);

    cv::Mat disparity16S, disparity8U;
    stereo->compute (left_gray, right_gray, disparity16S);

    disparity16S.convertTo (disparity8U, CV_8U, 255.0 / (numDisparities * 16));

    if (debug)
    {
      cv::imwrite ("./out/disparity.jpg", disparity8U);
      cv::Mat disparity_color;
      cv::applyColorMap (disparity8U, disparity_color, cv::COLORMAP_JET);
      cv::imwrite ("./out/disparity_color.jpg", disparity_color);
    }

    // Compute depth map
    cv::Mat disparity_float;
    disparity16S.convertTo(disparity_float, CV_32F, 1.0 / 16.0);

    depth_map = cv::Mat (disparity_float.size (), CV_32F);
    float fx = P1.at<double> (0, 0);
    float B = std::abs (P2.at<double> (0, 3)) / fx;

    for (int y = 0; y < disparity_float.rows; y++)
      for (int x = 0; x < disparity_float.cols; x++)
      {
        float d = disparity_float.at<float> (y, x);
        if (d > 0.1f)
          depth_map.at<float> (y, x) = fx * B / d;
        else
          depth_map.at<float> (y, x) = 0;
      }

    left_img = left_rect;
    right_img = right_rect;
  }
};

int main ()
{
  OV9715 ov9715 ("./stereo_calib.yml", "./in/stereo_room.jpg");
  ov9715.process_frame (true);
}