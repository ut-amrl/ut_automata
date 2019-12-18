// Copyright 2019 kvedder@seas.upenn.edu
// School of Engineering and Applied Sciences,
// University of Pennsylvania
//
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================

#include <gtest/gtest.h>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "math/geometry.h"
#include "math/line2d.h"

using geometry::Line;
using geometry::line2f;
using Eigen::Vector2f;

TEST(Line2D, Touching) {
  {
    const line2f l1(Vector2f(1, 4), Vector2f(4, 1));
    const line2f l2(Vector2f(6, 6), Vector2f(2.5, 2.5));
    EXPECT_TRUE(l1.Intersects(l2));
  }
}

TEST(Line2D, CollinearNoIntersection) {
  {
    const line2f l1(Vector2f(1, 1), Vector2f(2, 2));
    const line2f l2(Vector2f(3, 3), Vector2f(4, 4));
    EXPECT_FALSE(l1.Intersects(l2));
  }
}

TEST(Line2D, CollinearOverlap) {
  {
    const line2f l1(Vector2f(1, 1), Vector2f(3, 3));
    const line2f l2(Vector2f(2, 2), Vector2f(4, 4));
    EXPECT_TRUE(l1.Intersects(l2));
  }
}

TEST(Line2D, CollinearTouching) {
  {
    const line2f l1(Vector2f(1, 1), Vector2f(3, 3));
    const line2f l2(Vector2f(3, 3), Vector2f(4, 4));
    EXPECT_TRUE(l1.Intersects(l2));
  }
}

TEST(Line2D, JumboSuite) {
  EXPECT_TRUE(line2f(1.1, 0.1, 1.9, -0.1).Intersects(line2f(1, 0, 2, 0)));
  EXPECT_TRUE(line2f(1.1, 0.1, 1.9, 0).Intersects(line2f(1, 0, 2, 0)));
  EXPECT_FALSE(line2f(1.1, 0.1, 1.9, 0.001).Intersects(line2f(1, 0, 2, 0)));
  EXPECT_FALSE(line2f(0, 0.1, 1.1, -0.1).Intersects(line2f(1, 0, 2, 0)));
  EXPECT_FALSE(line2f(1, 1, 2, 2).Intersects(line2f(3, 3, 6, 6)));
  EXPECT_TRUE(line2f(0, 0, 0, 5).Intersects(line2f(0, 5, 6, 6)));
  EXPECT_TRUE(line2f(0, 0, 0, 5).Intersects(line2f(0, 4, 0, 6)));
  EXPECT_TRUE(line2f(0, 0, 0, 5).Intersects(line2f(0, 5, 0, 6)));
  EXPECT_FALSE(line2f(0, 0, 2, 3).Intersects(line2f(4, 6, 6, 9)));
  EXPECT_TRUE(line2f(1, 1, 2, 2).Intersects(line2f(1, 1, 2, 2)));
  EXPECT_FALSE(line2f(1, 1, 2, 2).Intersects(line2f(2, 1, 3, 2)));
  EXPECT_TRUE(line2f(5, 0, 5, 10).Intersects(line2f(0, 5, 10, 5)));
  EXPECT_FALSE(line2f(5, 0, 5, 10).Intersects(line2f(50, 50, 60, 60)));
  EXPECT_TRUE(line2f(-1, -1, 1, 1).Intersects(line2f(-2, -2, 3, 3)));
  EXPECT_FALSE(line2f(0, 0, 0, 5).Intersects(line2f(1, 1, 5, 5)));
  EXPECT_TRUE(line2f(0, 0, 0, 5).Intersects(line2f(0, 5, 0, 10)));
  EXPECT_FALSE(line2f(10, 10, 10, 10).Intersects(line2f(30, 4, 30, 10)));
  EXPECT_TRUE(line2f(0, 200, 300, 200).Intersects(line2f(100, 300, 100, 200)));
}
