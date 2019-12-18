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

#include "math/geometry.h"

TEST(Heading, AngleZero) {
  EXPECT_EQ(Eigen::Vector2f(1, 0), geometry::Heading(0.0f));
}

TEST(CheckLineLineCollision, Collides) {
  {
    Eigen::Vector2f p1(1.1, 0.1);
    Eigen::Vector2f p2(1.9, -0.1);
    Eigen::Vector2f l1(1, 0);
    Eigen::Vector2f l2(2, 0);
    EXPECT_TRUE(geometry::CheckLineLineCollision(p1, p2, l1, l2));
  }
  {
    Eigen::Vector2f p1(1.1, 0.1);
    Eigen::Vector2f p2(1.9, 0);
    Eigen::Vector2f l1(1, 0);
    Eigen::Vector2f l2(2, 0);
    EXPECT_TRUE(geometry::CheckLineLineCollision(p1, p2, l1, l2));
  }
  {
    Eigen::Vector2f p1(1.1, 0.1);
    Eigen::Vector2f p2(1.9, 0.001);
    Eigen::Vector2f l1(1, 0);
    Eigen::Vector2f l2(2, 0);
    EXPECT_FALSE(geometry::CheckLineLineCollision(p1, p2, l1, l2));
  }
  {
    Eigen::Vector2f p1(0, 0.1);
    Eigen::Vector2f p2(1.1, -0.1);
    Eigen::Vector2f l1(1, 0);
    Eigen::Vector2f l2(2, 0);
    EXPECT_FALSE(geometry::CheckLineLineCollision(p1, p2, l1, l2));
  }
  {
    const Eigen::Vector2f p1(1, 1);
    const Eigen::Vector2f p2(2, 2);
    const Eigen::Vector2f p3(3, 3);
    const Eigen::Vector2f p4(6, 6);
    EXPECT_FALSE(geometry::CheckLineLineCollision(p1, p2, p3, p4));
  }
  {
    const Eigen::Vector2f p1(0, 0);
    const Eigen::Vector2f p2(0, 5);
    const Eigen::Vector2f p3(0, 5);
    const Eigen::Vector2f p4(6, 6);
    EXPECT_TRUE(geometry::CheckLineLineCollision(p1, p2, p3, p4));
  }
  {
    const Eigen::Vector2f p1(0, 0);
    const Eigen::Vector2f p2(0, 5);
    const Eigen::Vector2f p3(0, 4);
    const Eigen::Vector2f p4(0, 6);
    EXPECT_TRUE(geometry::CheckLineLineCollision(p1, p2, p3, p4));
  }
  {
    const Eigen::Vector2f p1(0, 0);
    const Eigen::Vector2f p2(0, 5);
    const Eigen::Vector2f p3(0, 5);
    const Eigen::Vector2f p4(0, 6);
    EXPECT_TRUE(geometry::CheckLineLineCollision(p1, p2, p3, p4));
  }
  {
    const Eigen::Vector2f p1(0, 0);
    const Eigen::Vector2f p2(2, 3);
    const Eigen::Vector2f p3(4, 6);
    const Eigen::Vector2f p4(6, 9);
    EXPECT_FALSE(geometry::CheckLineLineCollision(p1, p2, p3, p4));
  }
  {
    const Eigen::Vector2f p1(1, 1);
    const Eigen::Vector2f p2(2, 2);
    const Eigen::Vector2f p3(1, 1);
    const Eigen::Vector2f p4(2, 2);
    EXPECT_TRUE(geometry::CheckLineLineCollision(p1, p2, p3, p4));
  }
  {
    const Eigen::Vector2f p1(1, 1);
    const Eigen::Vector2f p2(2, 2);
    const Eigen::Vector2f p3(2, 1);
    const Eigen::Vector2f p4(3, 2);
    EXPECT_FALSE(geometry::CheckLineLineCollision(p1, p2, p3, p4));
  }
  {
    const Eigen::Vector2f p1(5, 0);
    const Eigen::Vector2f p2(5, 10);
    const Eigen::Vector2f p3(0, 5);
    const Eigen::Vector2f p4(10, 5);
    EXPECT_TRUE(geometry::CheckLineLineCollision(p1, p2, p3, p4));
  }
  {
    const Eigen::Vector2f p1(5, 0);
    const Eigen::Vector2f p2(5, 10);
    const Eigen::Vector2f p3(50, 50);
    const Eigen::Vector2f p4(60, 60);
    EXPECT_FALSE(geometry::CheckLineLineCollision(p1, p2, p3, p4));
  }
  {
    const Eigen::Vector2f p1(-1, -1);
    const Eigen::Vector2f p2(1, 1);
    const Eigen::Vector2f p3(-2, -2);
    const Eigen::Vector2f p4(3, 3);
    EXPECT_TRUE(geometry::CheckLineLineCollision(p1, p2, p3, p4));
  }
  {
    const Eigen::Vector2f p1(0, 0);
    const Eigen::Vector2f p2(0, 5);
    const Eigen::Vector2f p3(1, 1);
    const Eigen::Vector2f p4(5, 5);
    EXPECT_FALSE(geometry::CheckLineLineCollision(p1, p2, p3, p4));
  }
  {
    const Eigen::Vector2f p1(0, 0);
    const Eigen::Vector2f p2(0, 5);
    const Eigen::Vector2f p3(0, 4);
    const Eigen::Vector2f p4(0, 10);
    EXPECT_TRUE(geometry::CheckLineLineCollision(p1, p2, p3, p4));
  }
  {
    const Eigen::Vector2f p1(10, 10);
    const Eigen::Vector2f p2(10, 10);
    const Eigen::Vector2f p3(30, 4);
    const Eigen::Vector2f p4(30, 10);
    EXPECT_FALSE(geometry::CheckLineLineCollision(p1, p2, p3, p4));
  }
  {
    const Eigen::Vector2f p1(0, 200);
    const Eigen::Vector2f p2(300, 200);
    const Eigen::Vector2f p3(100, 300);
    const Eigen::Vector2f p4(100, 200);
    EXPECT_TRUE(geometry::CheckLineLineCollision(p1, p2, p3, p4));
  }
}

TEST(LineLineIntersection, CollidesAndIntersection) {
  {
    const Eigen::Vector2f p1(-1, 0);
    const Eigen::Vector2f p2(1, 0);
    const Eigen::Vector2f p3(0, -1);
    const Eigen::Vector2f p4(0, 1);
    EXPECT_TRUE(geometry::CheckLineLineCollision(p1, p2, p3, p4));
    const Eigen::Vector2f intersection = geometry::LineLineIntersection(
        p1, p2, p3, p4);
    EXPECT_FLOAT_EQ(intersection.x(), 0);
    EXPECT_FLOAT_EQ(intersection.y(), 0);
  }
  {
    const Eigen::Vector2f p1(-1, -1);
    const Eigen::Vector2f p2(3, 3);
    const Eigen::Vector2f p3(2, -5);
    const Eigen::Vector2f p4(-2, 5);
    EXPECT_TRUE(geometry::CheckLineLineCollision(p1, p2, p3, p4));
    const Eigen::Vector2f intersection = geometry::LineLineIntersection(
        p1, p2, p3, p4);
    EXPECT_FLOAT_EQ(intersection.x(), 0);
    EXPECT_FLOAT_EQ(intersection.y(), 0);
  }
  {
    const Eigen::Vector2f p1(-5, -5);
    const Eigen::Vector2f p2(5, -5);
    const Eigen::Vector2f p3(0, -5.25);
    const Eigen::Vector2f p4(4.99, -5);
    EXPECT_TRUE(geometry::CheckLineLineCollision(p1, p2, p3, p4));
    const Eigen::Vector2f intersection = geometry::LineLineIntersection(
        p1, p2, p3, p4);
    EXPECT_FLOAT_EQ(intersection.x(), 4.99);
    EXPECT_FLOAT_EQ(intersection.y(), -5);
  }
}



