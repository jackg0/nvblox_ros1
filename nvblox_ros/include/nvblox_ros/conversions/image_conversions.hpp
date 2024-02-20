// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef NVBLOX_ROS__CONVERSIONS__IMAGE_CONVERSIONS_HPP_
#define NVBLOX_ROS__CONVERSIONS__IMAGE_CONVERSIONS_HPP_

#include <nvblox/nvblox.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace nvblox {
namespace conversions {

struct QMatrix
{
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    double tx{0.0};
    double cxPrime{0.0};
    bool initialized{false};

    Eigen::Vector3d reprojectPoint(const double &x, const double &y, const double &disparity)
    {
        const double xB = ((fy * tx) * x) + (-fy * cx * tx);
        const double yB = ((fx * tx) * y) + (-fx * cy * tx);
        const double zB = (fx * fy * tx);
        const double invB  = 1.0 / ((-fy * disparity) + (fy * (cx - cxPrime)));

        Eigen::Vector3d point{xB * invB, yB * invB, zB * invB};
        return point;
    }
};

// Convert camera info message to NVBlox camera object
Camera cameraFromMessage(const sensor_msgs::CameraInfo& camera_info);

Camera cameraFromMessages(const sensor_msgs::CameraInfo& left_camera_info,
                          const sensor_msgs::CameraInfo& right_camera_info);

QMatrix qMatFromMessages(const sensor_msgs::CameraInfo& left_camera_info,
                         const sensor_msgs::CameraInfo& right_camera_info);

// Convert image to depth frame object
bool depthImageFromImageMessage(const sensor_msgs::ImageConstPtr& image_msg,
                                DepthImage* depth_frame,
                                const std::optional<QMatrix>& Q);

bool colorImageFromImageMessage(const sensor_msgs::ImageConstPtr& image_msg,
                                ColorImage* color_image);

bool monoImageFromImageMessage(const sensor_msgs::ImageConstPtr& image_msg,
                               MonoImage* mono_image);

// Convert depth frame to image message.
void imageMessageFromDepthImage(const DepthImage& depth_frame,
                                const std::string& frame_id,
                                sensor_msgs::Image* image_msg);

// Convert color frame to image message.
void imageMessageFromColorImage(const ColorImage& color_image,
                                const std::string& frame_id,
                                sensor_msgs::Image* image_msg);

}  // namespace conversions
}  // namespace nvblox

#endif  // NVBLOX_ROS__CONVERSIONS__IMAGE_CONVERSIONS_HPP_
