#include "tri.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <random>


cv::Mat prompt_filename(){
    std::string filename;
    std::cout << "Enter a filename > ";
    std::cin >> filename;

    cv::Mat img = cv::imread(filename, cv::IMREAD_GRAYSCALE);
    while(img.empty()) {
        std::cerr << "Could not find the image '" << filename << "'" << std::endl;
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max());
        std::cout << "Enter a filename > ";
        std::cin >> filename;

        img = cv::imread(filename, cv::IMREAD_GRAYSCALE);
    }

    return img;
}


int main(int argc, char** argv) {
    cv::Mat img;
    if(argc > 1) {
        img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
        if(img.empty()) {
            std::cerr << "Could not find the image '" << argv[1] << "'" << std::endl;
            img = prompt_filename();
        }
    }
    else img = prompt_filename();
    cv::Mat workingImg = img.clone();
    cv::namedWindow("Original", cv::WINDOW_NORMAL);
    std::vector<cv::Point2i> points;
    segment_list segments;
    bool isGray = true;

    while(1) {
        bool breakOut = false;
        std::string save_name;

        imshow("Original", workingImg);
        char key = cvWaitKey(0);

        switch(key) {
            case '1':
                workingImg = canny(img);
                break;
            case '2':
                points = gen_points(workingImg, 10);
                segments = std::move(tri::radial(workingImg, points));
                workingImg = draw_img(segments, workingImg.size());
                break;
            case 'C': // Convert to color
                if(isGray) {
                    cv::cvtColor(workingImg, workingImg, CV_GRAY2BGR);
                    isGray = false;
                }
                break;
            case 'c': // Reset image
                workingImg = img.clone();
                isGray = true;
                break;
            case 'q': // Quit program
                breakOut = true;
                break;
            case 27: // Quit program with ESC key
                breakOut = true;
                break;
            case 'o': // Open new file
                img = prompt_filename();
                workingImg = img.clone();
                break;
           case 's': // Save the image with a filename
                std::cout << "Enter a name to save the file > ";
                std::cin >> save_name;
                imwrite(save_name + ".jpg", workingImg);
                std::cout << "File successfully written to " + save_name + ".jpg" << std::endl;
            default:
                break;
        }

        if(breakOut) break;
    }

    cv::destroyAllWindows();

    return 0;
}
