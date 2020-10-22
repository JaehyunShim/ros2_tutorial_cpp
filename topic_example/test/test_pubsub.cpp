// Copyright 2020 Jaehyun Shim
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

#include <cmath>
#include <memory>
#include <algorithm>
#include "gtest/gtest.h"
#include "costmap_queue/costmap_queue.hpp"
#include "costmap_queue/limited_costmap_queue.hpp"
#include "rclcpp/rclcpp.hpp"

using std::hypot;

nav2_costmap_2d::Costmap2D costmap(5, 5, 1.0, 0.0, 0.0);

TEST(CostmapQueue, basicQueue)
{
  costmap_queue::CostmapQueue q(costmap);
  int count = 0;
  q.enqueueCell(0, 0);
  while (!q.isEmpty()) {
    costmap_queue::CellData cell = q.getNextCell();
    EXPECT_EQ(cell.distance_, hypot(cell.x_, cell.y_));
    count++;
  }
  EXPECT_EQ(count, 25);
}

TEST(CostmapQueue, bigTest)
{
  nav2_costmap_2d::Costmap2D big_map(500, 500, 1.0, 0.0, 0.0);
  costmap_queue::CostmapQueue q(big_map);
  int count = 0;
  q.enqueueCell(0, 0);
  while (!q.isEmpty()) {
    costmap_queue::CellData cell = q.getNextCell();
    EXPECT_EQ(cell.distance_, hypot(cell.x_, cell.y_));
    count++;
  }
  EXPECT_EQ(count, 500 * 500);
}

TEST(CostmapQueue, linearQueue)
{
  costmap_queue::CostmapQueue q(costmap);
  int count = 0;
  q.enqueueCell(0, 0);
  q.enqueueCell(0, 1);
  q.enqueueCell(0, 2);
  q.enqueueCell(0, 3);
  q.enqueueCell(0, 4);
  while (!q.isEmpty()) {
    costmap_queue::CellData cell = q.getNextCell();
    EXPECT_EQ(cell.distance_, cell.x_);
    count++;
  }
  EXPECT_EQ(count, 25);
}

TEST(CostmapQueue, crossQueue)
{
  costmap_queue::CostmapQueue q(costmap);
  int count = 0;
  int xs[] = {1, 2, 2, 3};
  int ys[] = {2, 1, 3, 2};
  int N = 4;
  for (int i = 0; i < N; i++) {
    q.enqueueCell(xs[i], ys[i]);
  }

  while (!q.isEmpty()) {
    costmap_queue::CellData cell = q.getNextCell();
    double min_d = 1000;

    for (int i = 0; i < N; i++) {
      double dd = hypot(xs[i] - static_cast<float>(cell.x_), ys[i] - static_cast<float>(cell.y_));
      min_d = std::min(min_d, dd);
    }
    EXPECT_NEAR(cell.distance_, min_d, 0.00001);
    count++;
  }
  EXPECT_EQ(count, 25);
}

TEST(CostmapQueue, limitedQueue)
{
  costmap_queue::LimitedCostmapQueue q(costmap, 5);
  int count = 0;
  q.enqueueCell(0, 0);
  while (!q.isEmpty()) {
    costmap_queue::CellData cell = q.getNextCell();
    EXPECT_EQ(cell.distance_, hypot(cell.x_, cell.y_));
    count++;
  }
  EXPECT_EQ(count, 24);

  costmap_queue::LimitedCostmapQueue q2(costmap, 3);
  count = 0;
  q2.enqueueCell(0, 0);
  while (!q2.isEmpty()) {
    q2.getNextCell();
    count++;
  }
  EXPECT_EQ(count, 11);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
