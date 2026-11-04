/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#include "manager/node_manager.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rs_driver/macro/version.hpp>
#include <signal.h>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <thread>

#ifdef ROS_FOUND
#include <ros/ros.h>
#include <ros/package.h>
#elif ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#endif

using namespace robosense::lidar;

#ifdef ROS_FOUND
static void sigHandler(int sig)
{
  (void)sig;
  RS_MSG << "RoboSense-LiDAR-Driver is stopping....." << RS_REND;
  ros::shutdown();
}
#elif ROS2_FOUND
static volatile std::sig_atomic_t g_shutdown_requested = 0;

static void sigHandler(int sig)
{
  (void)sig;
  g_shutdown_requested = 1;
}
#endif

int main(int argc, char** argv)
{
#ifdef ROS_FOUND
  signal(SIGINT, sigHandler);  ///< ROS 1 is initialized with NoSigintHandler below.
#elif ROS2_FOUND
  signal(SIGINT, sigHandler);
  signal(SIGTERM, sigHandler);
#endif

  RS_TITLE << "********************************************************" << RS_REND;
  RS_TITLE << "**********                                    **********" << RS_REND;
  RS_TITLE << "**********    RSLidar_SDK Version: v" << RSLIDAR_VERSION_MAJOR 
    << "." << RSLIDAR_VERSION_MINOR 
    << "." << RSLIDAR_VERSION_PATCH << "     **********" << RS_REND;
  RS_TITLE << "**********                                    **********" << RS_REND;
  RS_TITLE << "********************************************************" << RS_REND;

#ifdef ROS_FOUND
  ros::init(argc, argv, "rslidar_sdk_node", ros::init_options::NoSigintHandler);
#elif ROS2_FOUND
  rclcpp::init(
    argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
#endif

  std::string config_path;

#ifdef ROS2_FOUND
  const char * config_override = std::getenv("RSLIDAR_CONFIG_PATH");
  if (config_override != nullptr && config_override[0] != '\0') {
    config_path = config_override;
  } else {
    config_path = ament_index_cpp::get_package_share_directory("rslidar_sdk") +
      "/config/config.yaml";
  }
#elif defined(RUN_IN_ROS_WORKSPACE)
  config_path = ros::package::getPath("rslidar_sdk") + "/config/config.yaml";
#else
  config_path = "config/config.yaml";
#endif

#ifdef ROS_FOUND
  ros::NodeHandle priv_hh("~");
  std::string path;
  priv_hh.param("config_path", path, std::string(""));
  if (!path.empty())
  {
    config_path = path;
  }
#endif

  YAML::Node config;
  try
  {
    config = YAML::LoadFile(config_path);
  }
  catch (...)
  {
    RS_ERROR << "The format of config file " << config_path 
      << " is wrong. Please check (e.g. indentation)." << RS_REND;
    return -1;
  }

  std::shared_ptr<NodeManager> demo_ptr = std::make_shared<NodeManager>();
  demo_ptr->init(config);
  demo_ptr->start();

  RS_MSG << "RoboSense-LiDAR-Driver is running....." << RS_REND;

#ifdef ROS_FOUND
  ros::spin();
#elif ROS2_FOUND
  while (g_shutdown_requested == 0 && rclcpp::ok())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  RS_MSG << "RoboSense-LiDAR-Driver is stopping....." << RS_REND;
  demo_ptr->stop();
  demo_ptr.reset();
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
#endif

  return 0;
}
