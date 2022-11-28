// This file is part of the Robotic Template Library (RTL), a C++
// template library for usage in robotic research and applications
// under the MIT licence:
//
// Copyright 2020 Brno University of Technology
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom
// the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
// OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
// OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// Contact person: Adam Ligocki <adam.ligocki@vutbr.cz>

#include <gtest/gtest.h>
#include "rtl/Unstable.h"
#include <utility>
#include <iostream>

#define error 1e-4

template<typename T, std::size_t N>
constexpr T array_sum( std::array<T,N> array) {
    T sum = 0;
    for (std::size_t i = 0; i < N; i++) {
        sum += array[i];
    }
    return sum;
}

template<typename T, std::size_t N>
constexpr T array_product( std::array<T,N> array) {
    T sum = 0;
    for (std::size_t i = 0; i < N; i++) {
        sum += array[i];
    }
    return sum;
}


TEST(t_occupancymap_tests, initial1D) {
    rtl::OccupancyMapND<1, float, float, 10> map{{1.0f}};
    auto counter = 0.0f;
    for (size_t i = 0 ; i < 10 ; i++) {
            map.setCell(counter++, {i});
    }

    for (size_t i = 0 ; i < 10 ; i++) {
            std::cout << map.getCell({i}) << " ";
    }
}


TEST(t_occupancymap_tests, initial2D) {
    rtl::OccupancyMapND<2, float, float, 10, 10> map{{1.0f, 1.0f}};
    auto counter = 0.0f;
    for (size_t i = 0 ; i < 10 ; i++) {
        for (size_t j = 0 ; j < 10 ; j++) {
            map.setCell(counter++, {i,j});
        }
    }

    for (size_t i = 0 ; i < 10 ; i++) {
        for (size_t j = 0 ; j < 10 ; j++) {
            std::cout << map.getCell({i,j}) << " ";
        }
        std::cout << std::endl;
    }
}


TEST(t_occupancymap_tests, initial3D) {
    rtl::OccupancyMap3Dd<10, 10, 10> map{{1.0f, 1.0f, 1.0f}};
    auto counter = 0.0f;
    for (size_t i = 0 ; i < 10 ; i++) {
        for (size_t j = 0 ; j < 10 ; j++) {
            for (size_t k = 0 ; k < 10 ; k++) {
                map.setCell(counter++, {i, j, k});
            }
        }
    }

    for (size_t i = 0 ; i < 10 ; i++) {
        for (size_t j = 0 ; j < 10 ; j++) {
            for (size_t k = 0 ; k < 10 ; k++) {
                std::cout << map.getCell({i, j, k}) << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
        std::cout << std::endl;
    }
}

TEST(t_occupancymap_tests, distance1D) {
    rtl::OccupancyMapND<1, float, float, 10> map{{1.0f}};
    EXPECT_EQ(map.euclidean_distance({0}, {9}), 9.0f);
    EXPECT_EQ(map.distanceByAxis({0}, {9}).at(0), 9.0f);
}


TEST(t_occupancymap_tests, distance2D) {
    rtl::OccupancyMapND<2, float, float, 10, 10> map{{1.0f, 0.5f}};
    EXPECT_NEAR(map.euclidean_distance({0, 0}, {9, 9}), 10.0623f, error);
    auto dist = map.distanceByAxis({0, 0}, {9, 9});
    EXPECT_EQ(dist.at(0), 9.0f);
    EXPECT_EQ(dist.at(1), 4.5f);
}


TEST(t_occupancymap_tests, distance3D) {
    rtl::OccupancyMapND<3, float, float, 10, 10, 10> map{{1.0f, 0.5f, 2.0f}};
    EXPECT_NEAR(map.euclidean_distance({0, 0, 0}, {9, 9, 9}), 20.62159f, error);
    auto dist = map.distanceByAxis({0, 0, 0}, {9, 9, 9});
    EXPECT_EQ(dist.at(0), 9.0f);
    EXPECT_EQ(dist.at(1), 4.5f);
    EXPECT_EQ(dist.at(2), 18.0f);
}


TEST(t_occupancymap_tests, coordinates1D) {
    rtl::OccupancyMapND<1, float, float, 10> map{{1.0f}};

    EXPECT_EQ(map.coordinatesToIndex({0.0f}).at(0), 0);
    EXPECT_EQ(map.coordinatesToIndex({0.1f}).at(0), 0);
    EXPECT_EQ(map.coordinatesToIndex({0.5f}).at(0), 0);
    EXPECT_EQ(map.coordinatesToIndex({0.99f}).at(0), 0);
    EXPECT_EQ(map.coordinatesToIndex({1.0f}).at(0), 1);
    EXPECT_EQ(map.coordinatesToIndex({4.5f}).at(0), 4);
    EXPECT_EQ(map.coordinatesToIndex({9.99f}).at(0), 9);
    EXPECT_EQ(map.coordinatesToIndex({10.0f}).at(0), 10);

    EXPECT_EQ(map.indexToCoordinates({0}).at(0), 0.5);
    EXPECT_EQ(map.indexToCoordinates({2}).at(0), 2.5);
    EXPECT_EQ(map.indexToCoordinates({5}).at(0), 5.5);
    EXPECT_EQ(map.indexToCoordinates({8}).at(0), 8.5);
    EXPECT_EQ(map.indexToCoordinates({9}).at(0), 9.5);
}


TEST(t_occupancymap_tests, coordinates2D) {
    rtl::OccupancyMapND<2, float, float, 10, 10> map{{1.0f, 0.5}};

    auto index = map.coordinatesToIndex({0.0f, 0.0f});
    EXPECT_EQ(index.at(0), 0); EXPECT_EQ(index.at(1), 0);

    index = map.coordinatesToIndex({2.0f, 3.0f});
    EXPECT_EQ(index.at(0), 2); EXPECT_EQ(index.at(1), 6);

    index = map.coordinatesToIndex({8.3f, 1.3});
    EXPECT_EQ(index.at(0), 8); EXPECT_EQ(index.at(1), 2);

    index = map.coordinatesToIndex({9.3f, 0.49});
    EXPECT_EQ(index.at(0), 9); EXPECT_EQ(index.at(1), 0);


    auto coords = map.indexToCoordinates({0, 0});
    EXPECT_EQ(coords.at(0), 0.5); EXPECT_EQ(coords.at(1), 0.25);

    coords = map.indexToCoordinates({5, 3});
    EXPECT_EQ(coords.at(0), 5.5); EXPECT_EQ(coords.at(1), 1.75);

    coords = map.indexToCoordinates({2, 8});
    EXPECT_EQ(coords.at(0), 2.5); EXPECT_EQ(coords.at(1), 4.25);

    coords = map.indexToCoordinates({7, 6});
    EXPECT_EQ(coords.at(0), 7.5); EXPECT_EQ(coords.at(1), 3.25);
}

TEST(t_occupancymap_tests, coordinates3D) {
    rtl::OccupancyMapND<3, float, float, 10, 10, 10> map{{1.0f, 0.5f, 2.0f}};

    auto index = map.coordinatesToIndex({0.0f, 0.0f, 0.0f});
    EXPECT_EQ(index.at(0), 0); EXPECT_EQ(index.at(1), 0); EXPECT_EQ(index.at(2), 0);

    index = map.coordinatesToIndex({5.0f, 2.0f, 8.0});
    EXPECT_EQ(index.at(0), 5); EXPECT_EQ(index.at(1), 4); EXPECT_EQ(index.at(2), 4);

    index = map.coordinatesToIndex({6.8, 1.3, 7.2});
    EXPECT_EQ(index.at(0), 6); EXPECT_EQ(index.at(1), 2); EXPECT_EQ(index.at(2), 3);

    index = map.coordinatesToIndex({1.1f, 0.3, 13.6});
    EXPECT_EQ(index.at(0), 1); EXPECT_EQ(index.at(1), 0); EXPECT_EQ(index.at(2), 6);


    auto coords = map.indexToCoordinates({0, 0, 0});
    EXPECT_EQ(coords.at(0), 0.5); EXPECT_EQ(coords.at(1), 0.25); EXPECT_EQ(coords.at(2), 1.0);

    coords = map.indexToCoordinates({1, 2, 3});
    EXPECT_EQ(coords.at(0), 1.5); EXPECT_EQ(coords.at(1), 1.25); EXPECT_EQ(coords.at(2), 7.0);

    coords = map.indexToCoordinates({7, 5, 6});
    EXPECT_EQ(coords.at(0), 7.5); EXPECT_EQ(coords.at(1), 2.75); EXPECT_EQ(coords.at(2), 13.0);

    coords = map.indexToCoordinates({9, 8, 1});
    EXPECT_EQ(coords.at(0), 9.5); EXPECT_EQ(coords.at(1), 4.25); EXPECT_EQ(coords.at(2), 3.0);
}

TEST(t_occupancymap_tests, neighbours1D) {
    rtl::OccupancyMapND<1, float, float, 10> map{{1.0f}};
    auto counter = 0.0f;
    for (size_t i = 0 ; i < 10 ; i++) {
        map.setCell(counter++, {i});
    }

    size_t index = 5;
    auto neighbours = map.directNeighbourCellIndexes({index});
    EXPECT_EQ(neighbours.size(), 2);
    EXPECT_EQ(neighbours.at(0).at(0), 4);
    EXPECT_EQ(neighbours.at(1).at(0), 6);
    EXPECT_EQ(map.getCell(neighbours.at(0)), 4);
    EXPECT_EQ(map.getCell(neighbours.at(1)), 6);

    index = 0;
    neighbours = map.directNeighbourCellIndexes({index});
    EXPECT_EQ(neighbours.size(), 1);
    EXPECT_EQ(neighbours.at(0).at(0), 1);
    EXPECT_EQ(map.getCell(neighbours.at(0)), 1);

    index = 9;
    neighbours = map.directNeighbourCellIndexes({index});
    EXPECT_EQ(neighbours.size(), 1);
    EXPECT_EQ(neighbours.at(0).at(0), 8);
    EXPECT_EQ(map.getCell(neighbours.at(0)), 8);
}


TEST(t_occupancymap_tests, neighbours2D) {
    rtl::OccupancyMapND<2, float, float, 10, 10> map{{1.0f, 1.0f}};
    auto counter = 0.0f;
    for (size_t i = 0 ; i < 10 ; i++) {
        for (size_t j = 0 ; j < 10 ; j++) {
            map.setCell(counter++, {i, j});
        }
    }

    auto neighbours = map.directNeighbourCellIndexes({5, 8});
    EXPECT_EQ(neighbours.size(), 4);
    EXPECT_EQ(neighbours.at(0).at(0), 4); EXPECT_EQ(neighbours.at(0).at(1), 8);
    EXPECT_EQ(neighbours.at(1).at(0), 6); EXPECT_EQ(neighbours.at(1).at(1), 8);
    EXPECT_EQ(neighbours.at(2).at(0), 5); EXPECT_EQ(neighbours.at(2).at(1), 7);
    EXPECT_EQ(neighbours.at(3).at(0), 5); EXPECT_EQ(neighbours.at(3).at(1), 9);

    EXPECT_EQ(map.getCell(neighbours.at(0)), 48);
    EXPECT_EQ(map.getCell(neighbours.at(1)), 68);
    EXPECT_EQ(map.getCell(neighbours.at(2)), 57);
    EXPECT_EQ(map.getCell(neighbours.at(3)), 59);


    neighbours = map.directNeighbourCellIndexes({0, 0});
    EXPECT_EQ(neighbours.size(), 2);
    EXPECT_EQ(neighbours.at(0).at(0), 1); EXPECT_EQ(neighbours.at(0).at(1), 0);
    EXPECT_EQ(neighbours.at(1).at(0), 0); EXPECT_EQ(neighbours.at(1).at(1), 1);

    EXPECT_EQ(map.getCell(neighbours.at(0)), 10);
    EXPECT_EQ(map.getCell(neighbours.at(1)), 1);


    neighbours = map.directNeighbourCellIndexes({0, 9});
    EXPECT_EQ(neighbours.size(), 2);
    EXPECT_EQ(neighbours.at(0).at(0), 1); EXPECT_EQ(neighbours.at(0).at(1), 9);
    EXPECT_EQ(neighbours.at(1).at(0), 0); EXPECT_EQ(neighbours.at(1).at(1), 8);

    EXPECT_EQ(map.getCell(neighbours.at(0)), 19);
    EXPECT_EQ(map.getCell(neighbours.at(1)), 8);


    neighbours = map.directNeighbourCellIndexes({9, 0});
    EXPECT_EQ(neighbours.size(), 2);
    EXPECT_EQ(neighbours.at(0).at(0), 8); EXPECT_EQ(neighbours.at(0).at(1), 0);
    EXPECT_EQ(neighbours.at(1).at(0), 9); EXPECT_EQ(neighbours.at(1).at(1), 1);

    EXPECT_EQ(map.getCell(neighbours.at(0)), 80);
    EXPECT_EQ(map.getCell(neighbours.at(1)), 91);


    neighbours = map.directNeighbourCellIndexes({9, 9});
    EXPECT_EQ(neighbours.size(), 2);
    EXPECT_EQ(neighbours.at(0).at(0), 8); EXPECT_EQ(neighbours.at(0).at(1), 9);
    EXPECT_EQ(neighbours.at(1).at(0), 9); EXPECT_EQ(neighbours.at(1).at(1), 8);

    EXPECT_EQ(map.getCell(neighbours.at(0)), 89);
    EXPECT_EQ(map.getCell(neighbours.at(1)), 98);
}


TEST(t_occupancymap_tests, neighbours3D) {
    rtl::OccupancyMapND<3, float, float, 10, 10, 10> map{{1.0f, 1.0f, 1.0f}};
    auto counter = 0.0f;
    for (size_t i = 0 ; i < 10 ; i++) {
        for (size_t j = 0 ; j < 10 ; j++) {
            for (size_t k = 0 ; k < 10 ; k++) {
                map.setCell(counter++, {i, j, k});
            }
        }
    }

    auto neighbours = map.directNeighbourCellIndexes({7, 3, 4});
    EXPECT_EQ(neighbours.size(), 6);
    EXPECT_EQ(neighbours.at(0).at(0), 6); EXPECT_EQ(neighbours.at(0).at(1), 3); EXPECT_EQ(neighbours.at(0).at(2), 4);
    EXPECT_EQ(neighbours.at(1).at(0), 8); EXPECT_EQ(neighbours.at(1).at(1), 3); EXPECT_EQ(neighbours.at(1).at(2), 4);
    EXPECT_EQ(neighbours.at(2).at(0), 7); EXPECT_EQ(neighbours.at(2).at(1), 2); EXPECT_EQ(neighbours.at(2).at(2), 4);
    EXPECT_EQ(neighbours.at(3).at(0), 7); EXPECT_EQ(neighbours.at(3).at(1), 4); EXPECT_EQ(neighbours.at(3).at(2), 4);
    EXPECT_EQ(neighbours.at(4).at(0), 7); EXPECT_EQ(neighbours.at(4).at(1), 3); EXPECT_EQ(neighbours.at(4).at(2), 3);
    EXPECT_EQ(neighbours.at(5).at(0), 7); EXPECT_EQ(neighbours.at(5).at(1), 3); EXPECT_EQ(neighbours.at(5).at(2), 5);

    EXPECT_EQ(map.getCell(neighbours.at(0)), 634);
    EXPECT_EQ(map.getCell(neighbours.at(1)), 834);
    EXPECT_EQ(map.getCell(neighbours.at(2)), 724);
    EXPECT_EQ(map.getCell(neighbours.at(3)), 744);
    EXPECT_EQ(map.getCell(neighbours.at(4)), 733);
    EXPECT_EQ(map.getCell(neighbours.at(5)), 735);

    neighbours = map.directNeighbourCellIndexes({0, 0, 0}); EXPECT_EQ(neighbours.size(), 3);
    neighbours = map.directNeighbourCellIndexes({0, 0, 9}); EXPECT_EQ(neighbours.size(), 3);
    neighbours = map.directNeighbourCellIndexes({0, 9, 0}); EXPECT_EQ(neighbours.size(), 3);
    neighbours = map.directNeighbourCellIndexes({0, 9, 9}); EXPECT_EQ(neighbours.size(), 3);
    neighbours = map.directNeighbourCellIndexes({9, 0, 0}); EXPECT_EQ(neighbours.size(), 3);
    neighbours = map.directNeighbourCellIndexes({9, 0, 9}); EXPECT_EQ(neighbours.size(), 3);
    neighbours = map.directNeighbourCellIndexes({9, 9, 0}); EXPECT_EQ(neighbours.size(), 3);
    neighbours = map.directNeighbourCellIndexes({9, 9, 9}); EXPECT_EQ(neighbours.size(), 3);

    neighbours = map.directNeighbourCellIndexes({5, 5, 0}); EXPECT_EQ(neighbours.size(), 5);
    neighbours = map.directNeighbourCellIndexes({5, 0, 0}); EXPECT_EQ(neighbours.size(), 4);
    neighbours = map.directNeighbourCellIndexes({5, 5, 9}); EXPECT_EQ(neighbours.size(), 5);
    neighbours = map.directNeighbourCellIndexes({5, 9, 9}); EXPECT_EQ(neighbours.size(), 4);

    auto all_neighbours = map.allNeighbourCellIndexes({1, 5, 8});
    EXPECT_EQ(all_neighbours.size(), 26);

    all_neighbours = map.allNeighbourCellIndexes({0, 0, 0});
    EXPECT_EQ(all_neighbours.size(), 7);
}


TEST(t_occupancymap_tests, neighbours4D) {

    rtl::OccupancyMapND<4, float, float, 10, 10, 10, 10> map{{1.0f, 1.0f, 1.0f, 1.0f}};

    auto neighbours = map.directNeighbourCellIndexes({4, 2, 6, 8});
    EXPECT_EQ(neighbours.size(), 8);

    auto all_neighbours = map.allNeighbourCellIndexes({2, 4, 6, 8});
    EXPECT_EQ(all_neighbours.size(), 80);

    all_neighbours = map.allNeighbourCellIndexes({0, 0, 0, 0});
    EXPECT_EQ(all_neighbours.size(), 15);
}


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}