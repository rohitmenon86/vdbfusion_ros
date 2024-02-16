// MIT License
//
// # Copyright (c) 2022 Saurabh Gupta, Ignacio Vizzo, Cyrill Stachniss, University of Bonn
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "Transform.hpp"
#include "vdbfusion/VDBVolume.h"
#include "vdbfusion_ros/save_vdb_volume.h"
#include <std_srvs/Empty.h>

namespace vdbfusion {
class VDBVolumeNode {
public:
    VDBVolumeNode();

private:
    VDBVolume InitVDBVolume();
    void Integrate(const sensor_msgs::PointCloud2& pcd);
    bool saveVDBVolume(vdbfusion_ros::save_vdb_volume::Request& path,
                       vdbfusion_ros::save_vdb_volume::Response& response);

    bool pubVDBVolumeAsPointCloud(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::ServiceServer srv_;
    ros::ServiceServer srv_pub_;
    ros::Publisher pub_cloud_;
    Transform tf_;
    ros::Duration timestamp_tolerance_;

private:
    VDBVolume vdb_volume_;

    // PointCloud Processing
    bool preprocess_;
    bool apply_pose_;
    float min_range_;
    float max_range_;

    // Triangle Mesh Extraction
    bool fill_holes_;
    float min_weight_;
};
}  // namespace vdbfusion
