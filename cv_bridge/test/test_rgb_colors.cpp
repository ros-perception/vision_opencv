// Copyright (c) 2018 Intel Corporation.
// All Rights Reserved.
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

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include "cv_bridge/rgb_colors.h"

TEST(RGBColors, testGetRGBColor)
{
  cv::Vec3d color;
  // red
  color = cv_bridge::rgb_colors::getRGBColor(cv_bridge::rgb_colors::RED);
  EXPECT_EQ(1, color[0]);
  EXPECT_EQ(0, color[1]);
  EXPECT_EQ(0, color[2]);
  // gray
  color = cv_bridge::rgb_colors::getRGBColor(cv_bridge::rgb_colors::GRAY);
  EXPECT_EQ(0.502, color[0]);
  EXPECT_EQ(0.502, color[1]);
  EXPECT_EQ(0.502, color[2]);
}
