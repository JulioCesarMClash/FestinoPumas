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
#include <limits>
#include <tug_cfg/configuration.h>
#include <tug_cfg/yaml_reader.h>
#include <tug_cfg_example/ExampleConfig.h>
#include <tug_cfg_example/test_helpers.h>

TEST(TestExampleConfig, testDefaultConstructor)
{
  tug_cfg_example::ExampleConfig config;
}

TEST(TestExampleConfig, testConstrainer)
{
  tug_cfg_example::ExampleConfig config;
  ASSERT_EQ(config.decay_rate, 0.3);
  config.decay_rate = 0;
  tug_cfg::constrain(config);
  ASSERT_EQ(config.decay_rate, 1e-9);  // Should be minimum.
}

TEST(TestExampleConfig, testLoadYaml)
{
  tug_cfg_example::ExampleConfig config;
  tug_cfg::YamlReader reader("../../../src/utils/tug_cfg_example/test/example1.yaml");
  tug_cfg::load(config, reader);
  tug_cfg_example::assertExample1LoadedCorrectly(config);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
