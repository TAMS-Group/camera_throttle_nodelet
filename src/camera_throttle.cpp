/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2024, University of Hamburg
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <camera_throttle_nodelet/CameraThrottleConfig.h>

namespace camera_throttle_nodelet
{

class CameraThrottleNodelet : public nodelet::Nodelet
{
  std::shared_ptr<image_transport::ImageTransport> it_in_, it_out_;
  image_transport::CameraSubscriber sub_;
  image_transport::CameraPublisher pub_;

  int skip_;
  int n_;

  // Dynamic reconfigure
  typedef camera_throttle_nodelet::CameraThrottleConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  std::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;

  void onInit() override;

  void cameraCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);

  void configCb(Config &config, uint32_t level);
};

void CameraThrottleNodelet::onInit()
{
  ros::NodeHandle& nh         = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  ros::NodeHandle nh_in (nh, "camera");
  ros::NodeHandle nh_out(nh, "camera_out");
  it_in_ .reset(new image_transport::ImageTransport(nh_in));
  it_out_.reset(new image_transport::ImageTransport(nh_out));

  // Read parameters
  private_nh.param("skip", skip_, 2);
  n_ = 0;

  // Set up dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(private_nh));
  ReconfigureServer::CallbackType f = boost::bind(&CameraThrottleNodelet::configCb, this, boost::placeholders::_1, boost::placeholders::_2);
  reconfigure_server_->setCallback([this](auto& cfg, auto lvl){ configCb(cfg, lvl); });

  pub_ = it_out_->advertiseCamera("camera_out",  1);

  image_transport::TransportHints sub_hints("raw", ros::TransportHints(), private_nh);
  sub_ = it_in_->subscribeCamera("camera_in", 1, &CameraThrottleNodelet::cameraCb, this, sub_hints);
}

void CameraThrottleNodelet::cameraCb(const sensor_msgs::ImageConstPtr& image_msg,
                                  const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  if (!n_++)
    pub_.publish(image_msg, info_msg);
}

void CameraThrottleNodelet::configCb(Config &config, uint32_t level)
{
  config_ = config;
  n_ = 0;
}

} // namespace camera_throttle_nodelet

// Register nodelet
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( camera_throttle_nodelet::CameraThrottleNodelet, nodelet::Nodelet)
