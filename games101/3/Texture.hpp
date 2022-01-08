//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include <algorithm>
#include <cmath>
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]) / 255.0f;
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        float u_img = u * width;
        float v_img = (1 - v) * height;

        float u_min = std::clamp(std::floor(u_img), 0.0f, width - 1.0f);
        float v_min = std::clamp(std::floor(v_img), 0.0f, width - 1.0f);
        float u_max = std::clamp(std::ceil(u_img), 0.0f, width - 1.0f);
        float v_max = std::clamp(std::ceil(v_img), 0.0f, height - 1.0f);

        float k = u_img - u_min;
        float m = v_img - v_min;

        int u0 = (int)u_min, v0 = (int)v_min;
        int u1 = (int)u_max, v1 = (int)v_min;
        int u2 = (int)u_min, v2 = (int)v_max;
        int u3 = (int)u_max, v3 = (int)v_max;

        auto color0 = image_data.at<cv::Vec3b>(v0, u0);  // 注意opencv的坐标是行、列
        auto color1 = image_data.at<cv::Vec3b>(v1, u1);
        auto color2 = image_data.at<cv::Vec3b>(v2, u2);
        auto color3 = image_data.at<cv::Vec3b>(v3, u3);

        auto color = (color0*(1 - k) + color1*k)*(1 - m) + (color2*(1 - k) + color3*k)*m;
        return Eigen::Vector3f(color[0], color[1], color[2]) / 255.0f;
    }

};
#endif //RASTERIZER_TEXTURE_H
