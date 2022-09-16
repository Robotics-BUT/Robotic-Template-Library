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

#include <rtl/Core.h>
#include <iostream>
#include <random>
#include <iomanip>

struct SimpleCell {
    float travelCost = 0;
};



int main(int argc, char* argv[]) {

    constexpr size_t map_size = 10;
    constexpr float cell_size = 1.0f;
    rtl::OccupancyMapND<2, SimpleCell> map({map_size, map_size}, {cell_size, cell_size});


    // Fill map with values
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<> dist(0.0f, 10.0f);
    for (size_t i = 0 ; i < map_size ; i++) {
        for (size_t j = 0 ; j < map_size ; j++) {
            map.setCell(SimpleCell{.travelCost = static_cast<float>(dist(rng))}, {i, j});
        }
    }


    // Read map data
    for (size_t i = 0 ; i < map_size ; i++) {
        for (size_t j = 0 ; j < map_size ; j++) {
            std::cout << std::setw(5) << std::setprecision(3) << map.getCell({i, j}).travelCost << "\t";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;


    // Coordinates -> Index
    std::array<float, 2> coordinates = {8.3, 5.2};
    auto index = map.coordinatesToIndex(coordinates);
    std::cout << "X axis ... coord " << coordinates.at(0) << " corresponds with " << index.at(0) << " index" << std::endl;
    std::cout << "Y axis ... coord " << coordinates.at(1) << " corresponds with " << index.at(1) << " index" << std::endl;
    std::cout << std::endl;


    // Index -> Coordinates
    index = {3, 7};
    coordinates = map.indexToCoordinates(index);
    std::cout << "X axis ... index " << index.at(0) << " corresponds with " << coordinates.at(0) << " coordinate (middle of the cell)" << std::endl;
    std::cout << "Y axis ... index " << index.at(1) << " corresponds with " << coordinates.at(1) << " coordinate (middle of the cell)" << std::endl;
    std::cout << std::endl;


    // Distance between cells estimation
    std::array<size_t, 2> cell_index_1 = {1, 3};
    std::array<size_t, 2> cell_index_2 = {9, 7};

    auto dist_by_axis = map.distanceByAxis(cell_index_1, cell_index_2);
    auto euclidean_dist = map.euclidean_distance(cell_index_1, cell_index_2);
    std::cout << "Cell 1 index: " << cell_index_1.at(0) << " " << cell_index_1.at(1) << std::endl;
    std::cout << "Cell 2 index: " << cell_index_2.at(0) << " " << cell_index_2.at(1) << std::endl;
    std::cout << "Cell dimensions: " << cell_size << " x " << cell_size << std::endl;
    std::cout << "Distance in X axis: " << dist_by_axis.at(0) << std::endl;
    std::cout << "Distance in Y axis: " << dist_by_axis.at(1) << std::endl;
    std::cout << "Euclidean distance: " << euclidean_dist << std::endl;
    std::cout << std::endl;

    return 0;
}