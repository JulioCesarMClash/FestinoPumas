/*
 * This file is part of the software provided by the Graz University of Technology AIS group.
 *
 * Copyright (c) 2017, Alexander Buchegger
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted  provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <tug_cfg_example/example_node.h>
#include <boost/thread/lock_guard.hpp>
#include <ros/init.h>
#include <tug_cfg/configuration.h>
#include <tug_cfg/ros_param_reader.h>

namespace tug_cfg_example
{
ExampleNode::ExampleNode()
  : nh_("~"), config_server_(config_)
{
  config_server_.setLock(mutex_);
  config_server_.setCallback(std::bind(&ExampleNode::reconfigure, this));
}

ExampleNode::~ExampleNode()
{
}

void ExampleNode::start()
{
  boost::lock_guard<boost::mutex> lock(mutex_);
  tug_cfg::RosParamReader reader(nh_);
  tug_cfg::load(config_, reader);
  config_server_.start(nh_);
  timer_ = nh_.createTimer(ros::Duration(5.0), &ExampleNode::update, this);
  ROS_INFO("Running!");
}

void ExampleNode::update(const ros::TimerEvent&)
{
  boost::lock_guard<boost::mutex> lock(mutex_);
  ++config_.num_engineers;
  config_server_.notify();
}

void ExampleNode::reconfigure()
{
  ROS_INFO("Reconfigure!");
  if (config_.insane_mode)
  {
    ROS_ERROR("I'm quite sure you don't know what you're doing.");
    config_.insane_mode = false;
  }
}
}  // namespace tug_cfg_example



int main(int argc, char** argv)
{
  ros::init(argc, argv, "tug_cfg_example");
  tug_cfg_example::ExampleNode node;
  node.start();
  ros::spin();
  return 0;
}
