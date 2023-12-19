//
// Created by Max Laser on 26.11.23.
//

#include "canny.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include "canny.h"
#include "average_lines.cpp"

/**
 * @brief Visualize lines, normals, and end points in a colored image.
 *
 * @param whitePixelCoordinates Vector of Point2f containing x and y values of detected white pixels.
 * @param coloredImage Colored input image.
 * @param normalLength Length of the normal vectors.
 * @param normalSideInformation Information if the normal values should be defined on the left or right side.
 */
using namespace cv;



/**
 * @brief Get trajectory information based on white pixel coordinates in a binary image.
 *
 * @param whitePixelCoordinates Vector of Point2f containing x and y values of detected white pixels.
 * @param lineLength Number of points used to approximate one line using linear regression.
 * @param normalLength Length of the normal vectors.
 * @param leftSide If true, normals will be on the left side of the lines; otherwise, on the right side.
 * @param binaryImage Binary input image.
 * @return Vector of Point2f containing x and y values of end points of normal vectors.
 */
std::vector<Point2f> getTrajectory(const std::vector<Point2f>& whitePixelCoordinates, int lineLength, int normalLength, bool leftSide,Mat& binarayImage) {
    // Ensure the input image is of the correct type
    CV_Assert(binarayImage.type() == CV_8UC3);

    // Create a copy of the colored image for visualization
    Mat resultImage = binarayImage.clone();

    // Calculate the number of lines to be approximated
    size_t numLines = whitePixelCoordinates.size() / lineLength;

    // Vector to store end points of normal vectors
    std::vector<Point2f> endPointNormals;

    // Visualize lines, normals, and end points
    for (size_t i = 0; i < numLines; ++i) {
        // Extract points for linear regression
        //std::cout << "start= " << i * lineLength << "    end= " << (i + 1) * lineLength << std::endl;
        std::vector<Point2f> linePoints(whitePixelCoordinates.begin() + i * lineLength, whitePixelCoordinates.begin() + (i + 1) * lineLength);
        line(resultImage, Point2i(linePoints.front().x, linePoints.front().y), Point2i(linePoints.back().x, linePoints.back().y), Scalar(255, 96, 208), 4);

        // Berechne die Steigung der Geraden
        double slope = static_cast<double>(linePoints.back().y - linePoints.front().y) / (linePoints.back().x - linePoints.front().x);
        // Berechne die Verschiebung für die neue Linie basierend auf der Steigung und der Länge
        int dx = static_cast<int>(lineLength / sqrt(1 + slope * slope));
        int dy = static_cast<int>(slope * dx);



        // Visualize the line points in red

        circle(resultImage, Point2i(linePoints.front().x, linePoints.front().y), 3, Scalar(0, 0, 255), 4);
        circle(resultImage, Point2i(linePoints.back().x, linePoints.back().y), 3, Scalar(0, 0, 255), 4);
        //imshow("Extract points for linear regression:", resultImage);
        //waitKey(0);
/*
        // Print the first element
        if (!linePoints.empty()) {
            const auto& firstPoint = linePoints.front();
            std::cout << "f: x= " << firstPoint.x << "    y= " << firstPoint.y << std::endl;
        }

// Print the last element
        if (!linePoints.empty()) {
            const auto& lastPoint = linePoints.back();
            std::cout << "l: x= " << lastPoint.x << "    y= " << lastPoint.y << std::endl;
        }*/

        // Perform linear regression to approximate a line
        Vec4f lineParams;
        fitLine(linePoints, lineParams, DIST_L2, 0, 0.01, 0.01);

        // Visualize the line in blue
        //line(resultImage, Point2f(lineParams[2] - 15 * lineParams[0], lineParams[3] - 25 * lineParams[1]),
        //     Point2f(lineParams[2] + 25  * lineParams[0], lineParams[3] + 25 * lineParams[1]), Scalar(255, 0, 0), 2);

        // Visualize the start point in yellow
        //circle(resultImage, linePoints.front(), 5, Scalar(0, 255, 255), -1);

        // Visualize the end point in yellow
        //circle(resultImage, linePoints.back(), 5, Scalar(100, 255, 100), -1);

        // Calculate the normal vector direction based on the side information
        //int dx = static_cast<int>(lineParams[0]);
        //int dy = static_cast<int>(lineParams[1]);



        // Normalize the normal vector
        double length = sqrt(dx * dx + dy * dy);
        std::cout << "l: x= " << length << std::endl;
        double scale = normalLength / length;

        // Calculate the normal vector
        Vec4i lineNormal;
        lineNormal[0] = static_cast<int>(lineParams[2]);
        lineNormal[1] = static_cast<int>(lineParams[3]);
        if (leftSide) {
             lineNormal[2] = lineNormal[0] - static_cast<int>(scale * dy);
             lineNormal[3] = lineNormal[1] + static_cast<int>(scale * dx);
            std::cout << "leftSide: " << leftSide << std::endl;
        }else{
            lineNormal[2] = lineNormal[0] + static_cast<int>(scale * dy);
            lineNormal[3] = lineNormal[1] - static_cast<int>(scale * dx);
        }
        // Visualize the normal vector
        line(resultImage, Point2i(lineNormal[0], lineNormal[1]), Point2i(lineNormal[2], lineNormal[3]), Scalar(96, 255, 128), 2);
        //std::cout << "l: x= " << lineNormal[2] << "    y= " << lineNormal[3] << std::endl;


        // Extract the end point of the normal vector
        Point2f endPointNormal(lineNormal[2], lineNormal[3]);


        // Visualize the end point as a cross in red
       line(resultImage, Point2i(endPointNormal.x - 5, endPointNormal.y - 5), Point2i(endPointNormal.x + 5, endPointNormal.y + 5), Scalar(0, 0, 255), 2);
       line(resultImage, Point2i(endPointNormal.x - 5, endPointNormal.y + 5), Point2i(endPointNormal.x + 5, endPointNormal.y - 5), Scalar(0, 0, 255), 2);

        endPointNormals.push_back(endPointNormal);
    }

    // Display the result image
    imshow("Result Image:", resultImage);
    waitKey(0);

    // Return the vector of end points
    return endPointNormals;
}


cv::Mat processImage(const cv::Mat& inputImage) {
    cv::Mat outputImage = inputImage.clone();  // Create a copy of the input image

    for (int y = 0; y < outputImage.rows; ++y) {
        for (int x = 0; x < outputImage.cols - 1; ++x) {
            if (outputImage.at<uchar>(y, x) && outputImage.at<uchar>(y, x + 1) && x + 20 < outputImage.cols) {
                // Summarize pixels within 1 to 20 pixels apart
                for (int i = 1; i <= 20 && x + i < outputImage.cols; ++i) {
                    outputImage.at<uchar>(y, x + i) = 100;
                    //std::cout << "1 x: "<< x << "y:" << y << std::endl;

                }

                // Make the current pixel black
                outputImage.at<uchar>(y, x) = 200;
                //std::cout << "0 x: "<< x << "y:" << y << std::endl;
            }
        }
    }

    return outputImage;
}

cv::Mat summarizeWhitePixels(Mat& inputImage) {
    cv::Mat binaryImage = inputImage.clone();
    // Ensure the input image is of the correct type
    CV_Assert(binaryImage.type() == CV_8UC1);

    // Threshold for summarizing white pixels
    int threshold = 20;

    // Iterate over each row in the image
    for (int y = 0; y < binaryImage.rows; y++) {
        int countWhitePixels = 0;
        int startIndex = -1;

        // Iterate over each pixel in the row
        for (int x = 0; x < binaryImage.cols; x++) {
            // Check if the current pixel is white
            if (binaryImage.at<uchar>(y, x) == 255) {
                countWhitePixels++;
                if (startIndex == -1) {
                    startIndex = x;
                }
            }
            else {
                // Process the white pixels found in the row
                if (countWhitePixels > 0) {
                    int endIndex = startIndex + countWhitePixels - 1;
                    int middleIndex = startIndex + countWhitePixels / 2;


                    // Summarize white pixels based on the rules
                    for (int i = startIndex; i <= endIndex; i++) {
                        if (countWhitePixels <= 2 || countWhitePixels >= threshold) {
                            binaryImage.at<uchar>(y, i) = 255;  // Set the middle pixel to white
                        }
                        else {
                            binaryImage.at<uchar>(y, i) = 0;  // Summarize to black
                        }
                    }
                    binaryImage.at<uchar>(y, middleIndex) = 255;
                }

                // Reset counters for the next set of pixels
                countWhitePixels = 0;
                startIndex = -1;
            }
        }

        // Handle the case where the last pixels in the row are white
        if (countWhitePixels > 0) {
            int endIndex = startIndex + countWhitePixels - 1;
            int middleIndex = startIndex + countWhitePixels / 2;

            // Summarize white pixels based on the rules
            for (int i = startIndex; i <= endIndex; i++) {
                if (countWhitePixels <= 2 || countWhitePixels >= threshold) {
                    binaryImage.at<uchar>(y, i) = 255;  // Set the middle pixel to white
                }
                else {
                    binaryImage.at<uchar>(y, i) = 0;  // Summarize to black
                }
            }
        }
    }
    return binaryImage;
}

std::vector<Point2f> searchWhitePixels(const Mat& binaryImage) {
    // Ensure the input image is of the correct type
    CV_Assert(binaryImage.type() == CV_8UC1);

    // Vector to store the coordinates of detected white pixels
    std::vector<Point2f> whitePixelCoordinates;

    // Initialize the search center to the middle of the image in x direction
    int centerX = binaryImage.cols / 2;
    int centerY = binaryImage.rows - 1;


    int done=0;
    //serch first pixel
    for(int currentY=centerY;currentY>=0&&done==0;currentY--){
        for (int i = 1; ((centerX+i) > 0) && ((centerX-i) <(binaryImage.cols-1))&&done==0 ; i++) {
            if (binaryImage.at<uchar>(currentY, centerX) == 255) {
                centerY=currentY;
                done=1;
            }
            if (binaryImage.at<uchar>(currentY, centerX-i) == 255) {
                centerX=centerX-i;
                centerY=currentY;
                done=1;
        } if (binaryImage.at<uchar>(currentY, centerX+i) == 255){
                centerX=centerX+i;
                centerY=currentY;
                done=1;
            }
        }
    }

    // Parameters for the circular search pattern
    int searchRadius = 40;  // Radius of the circular search pattern
    int stepSize = 1;       // Step size for angles in the circular pattern

    // Start the search from the found center
    while (centerY >= 0 && centerX >= 0 && centerX < binaryImage.cols && centerY < binaryImage.rows) {
        // Check if the pixel at the current center is white
        if (binaryImage.at<uchar>(centerY, centerX) == 255) {
            // Add the coordinates of the white pixel to the vector

            // Check if the coordinates are already in the vector
            Point2f currentPoint(centerX, centerY);
            if (std::find(whitePixelCoordinates.begin(), whitePixelCoordinates.end(), currentPoint) == whitePixelCoordinates.end()) {
                // Add the coordinates of the white pixel to the vector
                whitePixelCoordinates.push_back(currentPoint);
            }

            // Perform circular search in x and y directions
            for (int radius = stepSize; radius <= searchRadius; radius += stepSize) {
                for (int angle = 0; angle <= 180; angle += stepSize) {
                    // Calculate coordinates in positive x direction
                    int x1 = centerX + static_cast<int>(radius * cos(angle * CV_PI / 180));
                    int y1 = centerY - static_cast<int>(radius * sin(angle * CV_PI / 180));

                    // Calculate coordinates in negative x direction
                    int x2 = centerX - static_cast<int>(radius * cos(angle * CV_PI / 180));
                    int y2 = centerY - static_cast<int>(radius * sin(angle * CV_PI / 180));

                    // Check if the coordinates are within the image bounds
                    if (x1 >= 0 && x1 < binaryImage.cols && y1 >= 0 && y1 < binaryImage.rows) {
                        // Check if the pixel at the calculated coordinates is white
                        if (binaryImage.at<uchar>(y1, x1) == 255) {
                            Point2f currentPoint(x1, y1);
                            if (std::find(whitePixelCoordinates.begin(), whitePixelCoordinates.end(), currentPoint) == whitePixelCoordinates.end()) {
                                whitePixelCoordinates.push_back(currentPoint);
                            }
                        }
                    }

                    // Check if the coordinates are within the image bounds
                    if (x2 >= 0 && x2 < binaryImage.cols && y2 >= 0 && y2 < binaryImage.rows) {
                        // Check if the pixel at the calculated coordinates is white
                        if (binaryImage.at<uchar>(y2, x2) == 255) {
                            Point2f currentPoint(x2, y2);
                            if (std::find(whitePixelCoordinates.begin(), whitePixelCoordinates.end(), currentPoint) == whitePixelCoordinates.end()) {
                                whitePixelCoordinates.push_back(currentPoint);
                            }
                        }
                    }

                }
            }
            if (centerX==static_cast<int>(whitePixelCoordinates.back().x)&&centerY==static_cast<int>(whitePixelCoordinates.back().y)){
                return whitePixelCoordinates;
            }
            // Update the search center to the newly found white pixel
            centerX = static_cast<int>(whitePixelCoordinates.back().x);
            centerY = static_cast<int>(whitePixelCoordinates.back().y);

        } else {
            // If no white pixel is found at the current center, move upwards in the y-direction
            centerY--;
        }
    }

    return whitePixelCoordinates;
}




void detectLaneMarkings2(cv::Mat img) {
    // Convert the image to grayscale
    cv::Mat gray, blur, edges, summarized, img_binary_new, img_lines;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::imshow("gray", gray);
    cv::waitKey(0);
    // Apply Gaussian blur to the image
    cv::GaussianBlur(gray, blur, cv::Size(5, 5), 0, 0);
    cv::imshow("blur", blur);
    cv::waitKey(0);

    // Convert the grayscale image to binary
    cv::Mat img_binary;
    cv::threshold(blur, img_binary, tresh, 255, cv::THRESH_BINARY);//debug images thresh = 100, real image tresh=175
    cv::imshow("threshold", img_binary);
    cv::waitKey(0);


    //summarizeWhitePixels
    img_binary_new=summarizeWhitePixels(img_binary);
    cv::imshow("summarizeWhitePixels", img_binary_new);
    imwrite("/Users/maxlaser/Desktop/02_AF/code/test/img_binary_new.png", img_binary_new);
    cv::waitKey(0);


    // Apply the function to search for white pixels
    std::vector<Point2f> whitePixelCoordinates = searchWhitePixels(img_binary_new);
    // Visualize white pixels with pink circles
    img_lines=img.clone();
    for (const Point2f& pixel : whitePixelCoordinates) {
        circle(img_lines, Point(static_cast<int>(pixel.x), static_cast<int>(pixel.y)), 2, Scalar(255, 0, 255), -1);
    }
    cv::imshow("searchWhitePixels", img_lines);
    cv::waitKey(0);



    // Mark the detected white pixels in the original binary image
    for (const auto& point : whitePixelCoordinates) {
        // Mark the white pixel in grey (e.g., intensity value of 128)
        img_binary_new.at<uchar>(static_cast<int>(point.y), static_cast<int>(point.x)+20) = 255;
        std::cout << "x= " << point.x<<"    y= " << point.y<< std::endl;
    }
    std::cout << "whitePixelCoordinates.size() = " << whitePixelCoordinates.size() << std::endl;
    cv::imshow("img_binary new ", img_binary_new);
    //cv::waitKey(0);

    // Example usage of getTrajectory function
    std::vector<Point2f> endPointNormals = getTrajectory(whitePixelCoordinates, 20, 30, true, img);

    /*
    // Print the obtained end points
    for (const auto& endPoint : endPointNormals) {
        std::cout << "End Point: x=" << endPoint.x << ", y=" << endPoint.y << std::endl;
    }
     */



    // Use Canny edge detection to detect edges in the image
    cv::Canny(img_binary, edges, 50, 150);
    cv::imshow("edges", edges);
    //cv::waitKey(0);

    summarized= processImage(edges);
    cv::imshow("edges", summarized);
    //cv::waitKey(0);
    //cv::bitwise_and(edges, mask,  edges); //todo solve error

    // Use Hough transform to detect lines in the image
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI/180, 10, 5, 30);
    std::cout << "lines.size() = " << lines.size() << std::endl;

    // Separate lines into left, middle, and right based on their x-position
    std::vector<cv::Vec4i> left_lines, middle_lines, right_lines;


    cv::Mat img_copy= img.clone();
    // Draw the lines
    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::Vec4i l = lines[i];
        cv::line( img_copy, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(i*3,i*3,0), 3, cv::LINE_AA);
    }
    cv::imshow("img_copy", img_copy);
    //cv::waitKey(0);

    lines = summarizeLines(lines);

    // Draw the lines
    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::Vec4i l = lines[i];
        cv::line( img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,100,i*3), 3, cv::LINE_AA);
        if(i%10==0){//
            //cv::imshow("Lane Detection", img);
            //cv::waitKey(0);
        }
    }
    cv::imshow("Lane Detection", img);
    //cv::waitKey(0);

    for (cv::Vec4i line : lines) {
        int x1 = line[0], x2 = line[2];
        if (x1 < img.cols / 3 && x2 < img.cols / 3) {
            left_lines.push_back(line);
        } else if (x1 > 2 * img.cols / 3 && x2 > 2 * img.cols / 3) {
            right_lines.push_back(line);
        } else {
            middle_lines.push_back(line);
        }
    }

    for( size_t i = 0; i < left_lines.size(); i++ )
    {

        cv::Vec4i l = left_lines[i];
        cv::line( img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,100,100), 3, cv::LINE_AA);
    }

    for( size_t i = 0; i < middle_lines.size(); i++ )
    {
        cv::Vec4i l = middle_lines[i];
        cv::line( img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(50,50,0), 3, cv::LINE_AA);
    }

    for( size_t i = 0; i < right_lines.size(); i++ )
    {
        cv::Vec4i l = right_lines[i];
        cv::line( img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(50,0,50), 3, cv::LINE_AA);
    }

    // Average the position of each of the lines
    cv::Vec4i left_line = cv::mean(left_lines);
    cv::Vec4i middle_line = cv::mean(middle_lines);
    cv::Vec4i right_line = cv::mean(right_lines);

    // Draw the lines on the original image
    cv::line(img, cv::Point(left_line[0], left_line[1]), cv::Point(left_line[2], left_line[3]), cv::Scalar(0, 0, 255), 1);
    cv::line(img, cv::Point(middle_line[0], middle_line[1]), cv::Point(middle_line[2], middle_line[3]), cv::Scalar(0, 255, 0), 2);
    cv::line(img, cv::Point(right_line[0], right_line[1]), cv::Point(right_line[2], right_line[3]), cv::Scalar(0, 0, 255), 2);

    // Display the image with detected lanes
    cv::imshow("Lane Detection", img);
    //cv::waitKey(0);
}