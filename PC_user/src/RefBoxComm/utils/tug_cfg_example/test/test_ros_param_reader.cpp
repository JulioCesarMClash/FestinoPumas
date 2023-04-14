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
#include <gtest/gtest.h>
#include <iostream>
#include <limits>
#include <ros/init.h>
#include <tug_cfg/configuration.h>
#include <tug_cfg/log_error_handler.h>
#include <tug_cfg/ros_param_reader.h>
#include <tug_cfg_example/ExampleConfig.h>
#include <tug_cfg_example/test_helpers.h>

TEST(TestRosParamReader, testLoadRosParam1)
{
  tug_cfg_example::ExampleConfig config;
  tug_cfg::RosParamReader reader(ros::NodeHandle("~"), "example1");
  tug_cfg::load(config, reader);
  tug_cfg_example::assertExample1LoadedCorrectly(config);
}

TEST(TestRosParamReader, testLoadRosParam2)
{
  tug_cfg_example::ExampleConfig config;
  tug_cfg::RosParamReader reader(ros::NodeHandle("~example1"));
  tug_cfg::load(config, reader);
  tug_cfg_example::assertExample1LoadedCorrectly(config);
}

TEST(TestRosParamReader, testLoadRosParam3)
{
  tug_cfg_example::ExampleConfig config;
  tug_cfg::RosParamReader reader(ros::NodeHandle("/test_ros_param_reader/example1"));
  tug_cfg::load(config, reader);
  tug_cfg_example::assertExample1LoadedCorrectly(config);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_ros_param_reader");
  tug_cfg::ErrorHandler::set(tug_cfg::LogErrorHandler::createStreamHandler(std::cerr, "Warning: "));
  return RUN_ALL_TESTS();
}
