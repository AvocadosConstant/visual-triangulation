#include "gui.hpp"
#include "gui_consts.hpp"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <sstream>
#include <vector>
#include <opencv2/core.hpp>

/**
 * Note: OpenCV is used only for loading and saving the image.
 * Once the image is read in, we work with the pixels and
 * other characteristics of the image directly.
 */

int main(int argc, char **argv) {
  // Argument checks
  if (argc != 3) ERROR_EXIT("Usage: ./gui <0 (read in as grayscale) or 1 (read in as color)> <path-to-image>");
  std::istringstream ss(argv[1]);
  int color;
  if (!(ss >> color)) ERROR_EXIT("Invalid number, please choose 0 (grayscale) or 1 (color)");

  // Gui setup, track original and copy of selected image
  std::string mainWindowTitle = "CS 455 Term Project";
  cv::Mat orig, clone;

  // Extract filename
  std::string filename(argv[2]);
  size_t idx = filename.find_last_of(".");

  filename = filename.substr(0, idx);
  if (color >= 0) orig = cv::imread(argv[2], color);
  clone = orig.clone();

  bool running = true;

  cv::namedWindow(mainWindowTitle);

  while (running) {
    cv::imshow(mainWindowTitle, clone);
    char key = cvWaitKey(WAIT_TIME);
    switch (key) {
      // Quit gui
      case ESCAPE:
        running = false;
        break;
        // Reset to original image
      case SPACE:
        orig.copyTo(clone);
        break;
      case 's':
        std::vector<cv::Point2f> corners = detect_corners_shi_tomasi(clone, 500);
        clone = draw_points(clone, corners, 6);
        break;
    }
  }

  // Destructor for gui window
  cv::destroyAllWindows();

  return 0;
}