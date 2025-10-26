/**
 * @file OV9715.cpp
 * @brief Class for stereo camera
 * @details Run info:
 *            sudo apt install libopencv-dev
 *            g++ OV9715.cpp -o OV9715 `pkg-config --cflags --libs opencv4`
 *            ./OV9715
 * @cite Useful information from:
 * https://www.cs.cmu.edu/~16385/s17/Slides/13.1_Stereo_Rectification.pdf
 * https://www.cs.cmu.edu/~16385/s17/Slides/13.2_Stereo_Matching.pdf
 * https://learnopencv.com/camera-calibration-using-opencv/
 */

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cassert>

class OV9715
{
public:
  OV9715 (int port = 0)
  {
    stereo_cam = cv::VideoCapture (port);
  }

  /**
   * For image testing
   */
  OV9715 (std::string source)
  {
    stereo_cam = cv::VideoCapture (source);
  }

  void process_frame (bool debug = false, std::string source = "")
  {
    if (!source.empty ())
      stereo_cam = cv::VideoCapture (source);

    capture_frame (debug);
    build_depth_map (debug);
  }

private:
  cv::VideoCapture stereo_cam;
  
  // Intrinsics estimates (use .calibrate () for real)
  float max_depth_mm = 5000;
  float width_px = 2560;
  float height_px = 960;
  float baseline_mm = 60;
  float fx = 1050;
  float fy = 1050;
  float cx = width_px / 4;
  float cy = height_px / 2;
  cv::Mat intrinsic_mat = (cv::Mat_<double> (3,3) << fx, 0, cx,   // fx, 0, cx
                                                     0, fy, cy,   // 0, fy, cy
                                                     0, 0, 1);    // 0, 0, 1
  // Manual distortion coefficients
  cv::Mat distortion = (cv::Mat_<double>(1,5) << 
                        -0.5, // more negative to correct for barrel distortion
                        0.1, 0.0, 0.0, 0.0);

  // Images
  cv::Mat raw_frame;
  cv::Mat left_img;
  cv::Mat right_img;
  cv::Mat depth_map;

  void split_frame (const cv::Mat& frame, cv::Mat& left, cv::Mat& right)
  {
    left = frame (cv::Range (0, frame.rows), 
                  cv::Range (0, frame.cols / 2)).clone ();
    right = frame (cv::Range (0, frame.rows), 
                   cv::Range (frame.cols / 2, frame.cols)).clone ();
  }

  /**
   * Stores raw left and right images
   */
  void capture_frame (bool debug = false)
  {
    stereo_cam >> raw_frame;
    cv::Mat storage;
    split_frame (raw_frame, left_img, right_img);

    // Output images
    if (debug)
    {
      cv::imwrite ("./out/raw_left.jpg", left_img);
      cv::imwrite ("./out/raw_right.jpg", right_img);  
    }
  }

  /**
   * Build depth map from preprocessed frame
   */
  void build_depth_map (bool debug = false)
  {
    assert (!left_img.empty ());
    assert (!right_img.empty ());

    /***** UNDISTORT & RECTIFY *****/
    cv::Size img_size = left_img.size ();

    // Estimated R and T manually
    cv::Mat R;
    cv::Vec3d axis = {0, 0, 1};
    double theta_deg = 0.25;
    double theta = theta_deg * M_PI / 180.0;
    cv::Mat rvec = (cv::Mat_<double> (3,1) << axis[0]*theta, axis[1]*theta, axis[2]*theta);
    cv::Rodrigues (rvec, R);
    cv::Mat T = (cv::Mat_<double> (3, 1) << baseline_mm, -15, 0);
    
    cv::Mat R1, R2, P1, P2, Q;
    cv::Rect validRoi1, validRoi2;
    
    cv::stereoRectify (
      intrinsic_mat,  // Left camera matrix
      distortion,     // Left distortion
      intrinsic_mat,  // Right camera matrix (same camera)
      distortion,     // Right distortion
      img_size,       // Image size
      R,              // Rotation between cameras
      T,              // Translation between cameras
      R1, R2,         // Output rectification transforms
      P1, P2,         // Output projection matrices
      Q,              // Output disparity-to-depth mapping matrix
      cv::CALIB_ZERO_DISPARITY,  // Flag
      0,              // alpha (0 = crop to valid pixels, 1 = keep all)
      img_size,       // New image size
      &validRoi1,
      &validRoi2);
    
    if (debug)
    {
      std::cout << "Rectification computed" << std::endl;
      std::cout << "Valid ROI Left: " << validRoi1 << std::endl;
      std::cout << "Valid ROI Right: " << validRoi2 << std::endl;
    }
    
    // Compute rectification maps
    cv::Mat map1_left, map2_left, map1_right, map2_right;
    
    cv::initUndistortRectifyMap (
      intrinsic_mat, distortion, R1, P1, img_size,
      CV_32FC1, map1_left, map2_left
    );
    
    cv::initUndistortRectifyMap (
      intrinsic_mat, distortion, R2, P2, img_size,
      CV_32FC1, map1_right, map2_right
    );
    
    // Apply rectification
    cv::Mat left_rect, right_rect;
    cv::remap (left_img, left_rect, map1_left, map2_left, cv::INTER_LINEAR);
    cv::remap (right_img, right_rect, map1_right, map2_right, cv::INTER_LINEAR);
    
    if (debug)
    {      
      // Draw horizontal lines to verify rectification
      cv::Mat left_lines = left_rect.clone ();
      cv::Mat right_lines = right_rect.clone ();
      for (int y = 0; y < left_lines.rows; y += 50)
      {
        cv::line (left_lines, cv::Point(0, y), 
                cv::Point(left_lines.cols, y), cv::Scalar(0, 255, 0), 1);
        cv::line (right_lines, cv::Point(0, y), 
                cv::Point(right_lines.cols, y), cv::Scalar(0, 255, 0), 1);
      }
      cv::imwrite("./out/rectified_left_lines.jpg", left_lines);
      cv::imwrite("./out/rectified_right_lines.jpg", right_lines);
    }

    /***** BLOCK MATCHING *****/
    cv::Mat left_gray, right_gray;
    cv::cvtColor (left_rect, left_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor (right_rect, right_gray, cv::COLOR_BGR2GRAY);

    int minDisparity = 0;
    int numDisparities = 16 * 6;   // Must be divisible by 16
    int blockSize = 11;             // Odd number
    
    cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create (
      minDisparity,
      numDisparities,
      blockSize,
      8 * 3 * blockSize * blockSize,      // P1
      32 * 3 * blockSize * blockSize,     // P2
      1,                                  // disp12MaxDiff
      63,                                 // preFilterCap
      10,                                 // uniquenessRatio
      100,                                // speckleWindowSize
      32,                                 // speckleRange
      cv::StereoSGBM::MODE_SGBM_3WAY);
    
    cv::Mat disparity;
    stereo->compute (left_gray, right_gray, disparity);
    
    cv::Mat disparity_vis;
    double minVal, maxVal;
    cv::minMaxLoc (disparity, &minVal, &maxVal);
    disparity.convertTo (disparity_vis, CV_8U, 255.0 / (maxVal - minVal));
    
    if (debug)
    {
      cv::imwrite ("./out/disparity.jpg", disparity_vis);
      
      // Colored disparity
      cv::Mat disparity_color;
      cv::applyColorMap (disparity_vis, disparity_color, cv::COLORMAP_JET);
      cv::imwrite ("./out/disparity_color.jpg", disparity_color);
    }

    /***** DEPTH MAP *****/
    cv::Mat disparity_float;
    disparity.convertTo (disparity_float, CV_32F, 1.0/16.0);

    float f = intrinsic_mat.at<double> (0, 0);
    float B = baseline_mm;

    depth_map = cv::Mat::zeros (disparity_float.size (), CV_32F);
    float min_depth = std::numeric_limits<float>::max ();
    float max_depth = 0;
    for (int y = 0; y < disparity_float.rows; y++)
      for (int x = 0; x < disparity_float.cols; x++)
      {
        float disp = disparity_float.at<float> (y, x);
        if (disp > 1.0)
        {
          float depth = std::min ((f * B) / disp, max_depth_mm);
          depth_map.at<float> (y, x) = depth;
          min_depth = std::min (depth, min_depth);
          max_depth = std::max (depth, max_depth);
        }
      }

    if (debug)
    {
      std::cout << "Manual depth range: " << min_depth << " to " << max_depth<< " mm" << std::endl;
      cv::Mat depth_vis;
      cv::normalize (depth_map, depth_vis, 0, 255, cv::NORM_MINMAX, CV_8U);
      cv::applyColorMap (depth_vis, depth_vis, cv::COLORMAP_JET);
      cv::imwrite ("./out/depth_map.jpg", depth_vis);
    }
    
    left_img = left_rect;
    right_img = right_rect;
  }
};

int main ()
{
  OV9715 ov9715 = OV9715 ("./checker35mm/c8.jpg");
  ov9715.process_frame (true);
}