#include "gui_consts.hpp"
#include "gui.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

/**
 * Compresses the image using triangulation.
 * @param m     the image matrix to work with
 * @param size  the triangle size
 * @param color whether to work with grayscale or color
 */
void compress(cv::Mat &m, const int size, const int color) {
    // Make copy of original
    cv::Mat copy = m.clone();

    // Traverse image
    for (int i = 0; i < (copy.rows - (copy.rows % size)); i += size) {
        for (int j = 0; j < (copy.cols - (copy.cols % size)); j += size) {
            double sum = 0.0;
            double sumColor[3] = { 0.0 };
            int count = 0;
            // Get average color in top left triangular region
            for (int k = 0; k < size; k++) {
                for (int l = 0; l < size - k - 1; l++) {
                    if (color) {
                        for (int n = 0; n < 3; n++) sumColor[n] += (int)copy.at<cv::Vec3b>(i + k, j + l)[n];
                    } else {
                        sum += (int)copy.at<uchar>(i + k, j + l);
                    }
                    count++;
                }
            }
            int avgColor[3] = { 0 };
            int avg = 0;
            if (color) {
                for (int n = 0; n < 3; n++) avgColor[n] = std::round(sumColor[n] / count);
            } else {
                avg = std::round(sum / count);
            }

            // Recolor top left triangular region
            for (int k = 0; k < size; k++) {
                for (int l = 0; l < size - k - 1; l++) {
                    if (color) {
                        copy.at<cv::Vec3b>(i + k, j + l) = cv::Vec3b(avgColor[0], avgColor[1], avgColor[2]);
                    } else {
                        copy.at<uchar>(i + k, j + l) = avg;
                    }
                }
            }

            sum = 0.0;
            std::fill(sumColor, sumColor + 3, 0.0);
            count = 0;
            // Get average color in bottom right triangular region
            for (int k = size - 1; k >= 0; k--) {
                for (int l = size - 1; l >= size - k - 1; l--) {
                    if (color) {
                        for (int n = 0; n < 3; n++) sumColor[n] += (int)copy.at<cv::Vec3b>(i + k, j + l)[n];
                    } else {
                        sum += (int)copy.at<uchar>(i + k, j + l);
                    }
                    count++;
                }
            }
            if (color) {
                for (int n = 0; n < 3; n++) avgColor[n] = std::round(sumColor[n] / count);
            } else {
                avg = std::round(sum / count);
            }

            // Recolor bottom right triangular region
            for (int k = size - 1; k >= 0; k--) {
                for (int l = size - 1; l >= size - k - 1; l--) {
                    if (color) {
                        copy.at<cv::Vec3b>(i + k, j + l) = cv::Vec3b(avgColor[0], avgColor[1], avgColor[2]);
                    } else {
                        copy.at<uchar>(i + k, j + l) = avg;
                    }
                }
            }
        }
    }

    // Copy image matrix into m
    m = copy;
}

/**
 * Exports a "triangulated" image as .jpg and .cs455
 * @param m        the image matrix to work with
 * @param size     the triangle size for which to save in the .cs455 file
 * @param filename the filename to use as a prefix when exporting the images
 */
void exportImage(cv::Mat &m, const int size, const int color, std::string filename) {
    std::ofstream file;

    // Save .jpg file with max quality
    std::cout << "Exported same format file as: " << filename << "-triangulation.jpg" << std::endl;
    imwrite(filename + "-triangulation.jpg", m, (std::vector<int>) { CV_IMWRITE_JPEG_QUALITY, 100 });

    // Write data to .cs455 file
    std::cout << "Exported cs455 format file as: " << filename << "-triangulation.cs455" << std::endl;
    file.open(filename + "-triangulation.cs455", std::ios::out | std::ios::trunc | std::ios::binary);

    if (!file.is_open()) ERROR_EXIT("Error exporting file: could not create new file or open file for editing");

    // Write file width, height, and triangle size to binary file
    file.write(reinterpret_cast<const char *>(&m.rows), sizeof(m.rows));
    file.write(reinterpret_cast<const char *>(&m.cols), sizeof(m.cols));
    file.write(reinterpret_cast<const char *>(&size), sizeof(size));

    // Traverse across image and save region intensities to binary file
    for (int i = 0; i < m.rows; i += size) {
        for (int j = 0; j < m.cols; j += size) {
            if (color) {
                file.write((char *)(&m.at<cv::Vec3b>(i, j)), sizeof(cv::Vec3b));
                file.write((char *)(&m.at<cv::Vec3b>(i + size - 1, j + size - 1)), sizeof(cv::Vec3b));
            } else {
                file.write(reinterpret_cast<char*>(&m.at<uchar>(i, j)), sizeof(uchar));
                file.write(reinterpret_cast<char*>(&m.at<uchar>(i + size - 1, j + size - 1)), sizeof(uchar));
            }
        }
    }
    file.close();
}

/**
 * Imports the image from the given file path into the given matrix.
 * @param m        the image matrix to work with
 * @param filename the filename for which to import the image
 */
void importImage(cv::Mat &m, const int color, std::string filename) {
    std::ifstream file;
    file.open(filename, std::ios::in | std::ios::binary);
    if (!file) ERROR_EXIT("Error importing file: could not open file for reading");

    int rows, cols, size;
    uchar color1, color2;
    cv::Vec3b colorV1, colorV2;

    // Write file width, height, and region size to binary file
    file.read((char*)&rows, sizeof(int));
    file.read((char*)&cols, sizeof(int));
    file.read((char*)&size, sizeof(int));

    // Work with a draft matrix
    cv::Mat copy;
    if (color) {
        copy = cv::Mat(rows, cols, CV_8UC3);
    } else {
        copy = cv::Mat(rows, cols, CV_8U);
    }
    copy.setTo(cv::Scalar::all(0));

    // Traverse across image and save region intensities to binary file
    for (int i = 0; i < copy.rows; i += size) {
        for (int j = 0; j < copy.cols; j += size) {
            if (file.eof()) {
                file.close();
                return;
            }
            if (color) {
                file.read((char*)&colorV1, sizeof(cv::Vec3b));
                file.read((char*)&colorV2, sizeof(cv::Vec3b));
            } else {
                file.read((char*)&color1, sizeof(uchar));
                file.read((char*)&color2, sizeof(uchar));
            }

            // Recolor top left triangular region
            for (int k = 0; k < size && i + k < copy.rows; k++) {
                for (int l = 0; l < size - k - 1 && j + l < copy.cols; l++) {
                    if (color) {
                        copy.at<cv::Vec3b>(i + k, j + l) = colorV1;
                    } else {
                        copy.at<uchar>(i + k, j + l) = color1;
                    }
                }
            }

            // Recolor bottom right triangular region
            for (int k = size - 1; k >= 0 && i + k < copy.rows; k--) {
                for (int l = size - 1; l >= size - k - 1 && j + l < copy.cols; l--) {
                    if (color) {
                        copy.at<cv::Vec3b>(i + k, j + l) = colorV2;
                    } else {
                        copy.at<uchar>(i + k, j + l) = color2;
                    }
                }
            }
        }
    }
    file.close();

    // Save draft to image matrix
    m = copy.clone();
}
