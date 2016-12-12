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

  // Import image if .cs455 extension detected
  if (filename.substr(idx + 1) == "cs455") {
      importImage(orig, color, filename);
      orig.copyTo(clone);
  } else {
      filename = filename.substr(0, idx);
      // CV_LOAD_IMAGE_GRAYSCALE = 0, CV_LOAD_IMAGE_COLOR > 0
      if (color >= 0) orig = cv::imread(argv[2], color);
      clone = orig.clone();
  }

  int size = 0;
  cv::namedWindow(mainWindowTitle);

  std::vector<cv::Point2f> corners;

  bool running = true;
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
      // Apply compression with triangulation
      case COMPRESS:
        orig.copyTo(clone);
        std::cout << "Enter a positive integer size:" << std::endl;
        std::cin >> size;
        if (size - 1 >= 2) compress(clone, size, color);
        break;
      case EXPORT:
        exportImage(clone, size, color, filename);
        break;
      case 'r':
        corners = detect_corners_random_edge(clone, 500);
        orig.copyTo(clone);
        clone = draw_points(clone, corners, 6);
        break;
      case 's':
        corners = detect_corners_shi_tomasi(clone, 500);
        orig.copyTo(clone);
        clone = draw_points(clone, corners, 6);
        break;
    }
  }

  // Destructor for gui window
  cv::destroyAllWindows();

  return 0;
}
