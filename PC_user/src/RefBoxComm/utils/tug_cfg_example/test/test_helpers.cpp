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
#include <tug_cfg_example/test_helpers.h>
#include <gtest/gtest.h>
#include <limits>

namespace tug_cfg_example
{
void assertExample1LoadedCorrectly(const tug_cfg_example::ExampleConfig& config)
{
  ASSERT_EQ(config.insane_mode, false);
  ASSERT_EQ(config.decay_rate, 0.5);
  ASSERT_EQ(config.num_engineers, 2);
  ASSERT_EQ(config.engineer_names.size(), 2);
  ASSERT_EQ(config.engineer_names.at(1), "Al");
  ASSERT_EQ(config.engineer_names.at(2), "Bert");
  ASSERT_EQ(config.var_name, "bar");
  ASSERT_EQ(config.pseudo_random_numbers.size(), 3);
  ASSERT_EQ(config.pseudo_random_numbers.at(0), 42);
  ASSERT_EQ(config.pseudo_random_numbers.at(1), 666);
  ASSERT_EQ(config.pseudo_random_numbers.at(2), 13);
  ASSERT_EQ(config.max_speed, std::numeric_limits<double>::infinity());
  ASSERT_EQ(config.covariance.size(), 2);
  ASSERT_EQ(config.covariance.at(0).size(), 2);
  ASSERT_EQ(config.covariance.at(0).at(0), 1);
  ASSERT_EQ(config.covariance.at(0).at(1), 2);
  ASSERT_EQ(config.covariance.at(1).size(), 2);
  ASSERT_EQ(config.covariance.at(1).at(0), 3);
  ASSERT_EQ(config.covariance.at(1).at(1), 4);
  ASSERT_EQ(config.extra_params.size(), 2);
  ASSERT_EQ(config.extra_params.at("a"), "b");
  ASSERT_EQ(config.extra_params.at("c"), "d");
}
}  // namespace tug_cfg_example
