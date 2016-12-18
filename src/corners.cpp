#include "gui.hpp"
#include <iostream>
#include <algorithm>
#include <iomanip>

cv::Mat draw_points(cv::Mat image, std::vector<cv::Point2f> points, int radius) {
  for( int i = 0; i < points.size(); i++ ) {
    cv::circle(
      image, points[i], radius, cv::Scalar(
        255, 255, 255
      ), -1, 8, 0
    );
  }
  return image;
}

std::vector<cv::Point2f> detect_corners_random_edge(cv::Mat image, int maxCorners) {
  // Reduce noise with a kernel 3x3
  cv::blur(image, image, cv::Size(3,3));

  /// Canny detector
  cv::Canny(image, image, 160, 200, 3);

  std::vector<cv::Point2f> corners;

  for (int i = 0; i < image.cols; i++ ) {
    for (int j = 0; j < image.rows; j++) {
      if (image.at<uchar>(j, i) >0) {
        corners.push_back(cv::Point2f(i, j));
      }
    }
  }
  std::random_shuffle(corners.begin(), corners.end());
  corners = std::vector<cv::Point2f>(corners.begin(), corners.begin() + maxCorners - 4);

  // Push 4 real corners of image
  //corners.push_back(cv::Point2f(0,0));
  //corners.push_back(cv::Point2f(image.cols, 0));
  //corners.push_back(cv::Point2f(0, image.rows));
  //corners.push_back(cv::Point2f(image.cols, image.rows));

  return corners;
}

std::vector<cv::Point2f> detect_corners_shi_tomasi(cv::Mat image, int maxCorners) {
  // Make sure maxCorners is positive
  maxCorners = (maxCorners < 1) ? 1 : maxCorners;

  std::vector<cv::Point2f> corners;
  double qualityLevel = 0.01;
  double minDistance = 10;
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  // Shi-Tomasi Corner Detection
  cv::goodFeaturesToTrack(
      image,
      corners,
      maxCorners - 4,
      qualityLevel,
      minDistance,
      cv::Mat(),
      blockSize,
      useHarrisDetector,
      k );

  // Push 4 real corners of image
  //corners.push_back(cv::Point2f(0,0));
  //corners.push_back(cv::Point2f(image.cols, 0));
  //corners.push_back(cv::Point2f(0, image.rows));
  //corners.push_back(cv::Point2f(image.cols, image.rows));

  return corners;
}
