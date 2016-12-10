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
        importImage(orig, filename);
    } else {
        filename = filename.substr(0, idx);
        // CV_LOAD_IMAGE_GRAYSCALE = 0, CV_LOAD_IMAGE_COLOR > 0
        if (color >= 0) orig = cv::imread(argv[2], color);
    }
    clone = orig.clone();

    /**
     * Gui consistently monitors for key presses
     * until user clicks escape or uses CTRL + C
     * to force quit
     */
    bool running = true;

    // Set up size variable for compression
    int size = 5;

    char response;
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
            // Apply compression with triangulation
            case COMPRESS:
                response = cvWaitKey(WAIT_TIME * 1000);
                if (response == 'u') {
                    orig.copyTo(clone);
                    size += 75;
                    compress(clone, size);
                } else if (response == 'd') {
                    orig.copyTo(clone);
                    size -= 25;
                    if (size - 1 >= 2) compress(clone, size);
                } else if (response == ' ') {
                    exportImage(clone, size, filename);
                } else {
                    ERROR_CONT("Incorrect key pressed for compression using triangulation");
                }
                break;
        }
    }

    // Destructor for gui window
    cv::destroyAllWindows();

    return 0;
}
