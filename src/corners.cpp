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

std::vector<cv::Point2f> detect_corners_shi_tomasi(cv::Mat image, int maxCorners) {
  if( maxCorners < 1 ) { maxCorners = 1; }

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
      maxCorners,
      qualityLevel,
      minDistance,
      cv::Mat(),
      blockSize,
      useHarrisDetector,
      k );

  // Push 4 real corners of image
  corners.push_back(cv::Point2f(0,0));
  corners.push_back(cv::Point2f(image.cols, 0));
  corners.push_back(cv::Point2f(0, image.rows));
  corners.push_back(cv::Point2f(image.cols, image.rows));

  std::cout<<"Number of corners detected: "<<corners.size()<<std::endl;
  return corners;
}
