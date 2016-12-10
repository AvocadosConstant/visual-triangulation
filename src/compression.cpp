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
 * @param m    the image matrix to work with
 * @param size the triangle size
 */
void compress(cv::Mat &m, const int size) {
    // Make copy of original
    cv::Mat copy = m.clone();

    // Traverse image
    for (int i = 0; i < (copy.rows - (copy.rows % size)); i += size) {
        for (int j = 0; j < (copy.cols - (copy.cols % size)); j += size) {
            double sum = 0.0;
            int count = 0;
            // Get average color in top left triangular region
            for (int k = 0; k < size; k++) {
                for (int l = 0; l < size - k - 1; l++) {
                    sum += (int)copy.at<uchar>(i + k, j + l);
                    count++;
                }
            }
            int avg = std::round(sum / count);

            // Recolor top left triangular region
            for (int k = 0; k < size; k++) {
                for (int l = 0; l < size - k - 1; l++) {
                    copy.at<uchar>(i + k, j + l) = avg;
                }
            }

            sum = 0.0;
            count = 0;
            // Get average color in bottom right triangular region
            for (int k = size - 1; k >= 0; k--) {
                for (int l = size - 1; l >= size - k - 1; l--) {
                    sum += (int)copy.at<uchar>(i + k, j + l);
                    count++;
                }
            }
            avg = std::round(sum / count);

            // Recolor bottom right triangular region
            for (int k = size - 1; k >= 0; k--) {
                for (int l = size - 1; l >= size - k - 1; l--) {
                    copy.at<uchar>(i + k, j + l) = avg;
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
void exportImage(cv::Mat &m, const int size, std::string filename) {
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
            file.write(reinterpret_cast<char*>(&m.at<uchar>(i, j)), sizeof(uchar));
            file.write(reinterpret_cast<char*>(&m.at<uchar>(i + size - 1, j + size - 1)), sizeof(uchar));
        }
    }
    file.close();
}

/**
 * Imports the image from the given file path into the given matrix.
 * @param m        the image matrix to work with
 * @param filename the filename for which to import the image
 */
void importImage(cv::Mat &m, std::string filename) {
    std::ifstream file;
    file.open(filename, std::ios::in | std::ios::binary);
    if (!file) ERROR_EXIT("Error importing file: could not open file for reading");

    int rows, cols, size;
    uchar color1, color2;

    // Write file width, height, and region size to binary file
    file.read((char*)&rows, sizeof(int));
    file.read((char*)&cols, sizeof(int));
    file.read((char*)&size, sizeof(int));

    // Work with a draft matrix
    cv::Mat copy(rows, cols, CV_8U);
    copy.setTo(cv::Scalar::all(0));

    // Traverse across image and save region intensities to binary file
    for (int i = 0; i < copy.rows; i += size) {
        for (int j = 0; j < copy.cols; j += size) {
            if (file.eof()) {
                file.close();
                return;
            }
            file.read((char*)&color1, sizeof(uchar));
            file.read((char*)&color2, sizeof(uchar));

            // Recolor top left triangular region
            for (int k = 0; k < size && i + k < copy.rows; k++) {
                for (int l = 0; l < size - k - 1 && j + l < copy.cols; l++) {
                    copy.at<uchar>(i + k, j + l) = color1;
                }
            }

            // Recolor bottom right triangular region
            for (int k = size - 1; k >= 0 && i + k < copy.rows; k--) {
                for (int l = size - 1; l >= size - k - 1 && j + l < copy.cols; l--) {
                    copy.at<uchar>(i + k, j + l) = color2;
                }
            }
        }
    }
    file.close();

    // Save draft to image matrix
    m = copy.clone();
}
