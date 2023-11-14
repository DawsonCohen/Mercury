#ifndef __VIDEO_MANAGER_H__
#define __VIDEO_MANAGER_H__

#include <opencv2/opencv.hpp>
#include <glad/glad.h>
#include <filesystem>
#include <regex>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <sys/stat.h>

class VideoWriter {
public:
    VideoWriter(const std::string& filename, float fps, int width, int height) {
        video.open(filename, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, cv::Size(width, height));
        if (!video.isOpened()) {
            std::cerr << "ERROR: VideoWriter Could not open file " << filename << std::endl;
            exit(1);
        }
    }

    VideoWriter() {};

    ~VideoWriter() {
        video.release();
    }
    
    void writeFrame(const cv::Mat& frame) {
        if (!video.isOpened()) {
            std::cerr << "ERROR: VideoWriter could not write to file " << std::endl;
            exit(1);
        }

        video << frame;
    }

private:
    cv::VideoWriter video;
    int width;
    int height;
};

#endif