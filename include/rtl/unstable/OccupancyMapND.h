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

#ifndef ROBOTICTEMPLATELIBRARY_OCCUPANCYMAPND_H
#define ROBOTICTEMPLATELIBRARY_OCCUPANCYMAPND_H

#include <array>
#include <numeric>
#include <cmath>

namespace rtl
{
    using indexDType = size_t;

    template<size_t dim, typename CellType, typename distanceDType, size_t... gridSize>
    class OccupancyMapND_common {

        static constexpr size_t memorySize = (gridSize * ...);
        static_assert(dim != 0, "Occupancy Map has to be non-zero dimension");
        static_assert(dim == sizeof...(gridSize), "Invalid number of gridSize variadic arguments");

    public:
        OccupancyMapND_common() = delete;
        explicit OccupancyMapND_common(const std::array<distanceDType, dim>& cellSize)
                : cellSize_{cellSize}{
        }

        [[nodiscard]] const CellType& getCell(const std::array<indexDType, dim>& index) const {
            auto i = indexTo1D(index);
            return occGrid_.at(i);
        }

        void setCell(const CellType& cell, const std::array<indexDType, dim>& index) {
            auto i = indexTo1D(index);
            occGrid_.at(i) = cell;
        }

        [[nodiscard]] std::array<distanceDType, dim> indexToCoordinates(const std::array<indexDType, dim>& index) const {
            std::array<distanceDType, dim> outputCoordinates;
            for(size_t i = 0 ; i < dim ; i+=1) {
                outputCoordinates.at(i) = cellSize_.at(i) * (index.at(i) + 0.5);
            }
            return outputCoordinates;
        }

        [[nodiscard]] std::array<indexDType, dim> coordinatesToIndex(const std::array<distanceDType, dim>& coordinates) const {
            std::array<indexDType, dim> outputIndex;
            for(size_t i = 0 ; i < dim ; i+=1) {
                outputIndex.at(i) = static_cast<indexDType>(std::floor(coordinates.at(i) / cellSize_.at(i)));
            }
            return outputIndex;
        }

        distanceDType euclidean_distance(const std::array<indexDType, dim>& i1, const std::array<indexDType, dim>& i2) const {
            auto dist = distanceByAxis(i1, i2);
            distanceDType sum = 0;
            for (const auto& d : dist) {
                sum += std::pow(d, 2);
            }
            return std::sqrt(sum);
        }

        [[nodiscard]] std::array<distanceDType, dim> distanceByAxis(const std::array<indexDType, dim>& i1, const std::array<indexDType, dim>& i2) const {
            std::array<distanceDType, dim> outputDistance;
            for(size_t i = 0 ; i < dim ; i+=1) {
                outputDistance.at(i) = cellSize_.at(i) * (i2.at(i) - i1.at(i));
            }
            return outputDistance;
        }

        std::vector<std::array<indexDType, dim>> directNeighbourCellIndexes(const std::array<indexDType, dim>& index) {
            std::vector<std::array<indexDType, dim>> neighbours;
            neighbours.reserve(2*dim);

            for (size_t d = 0 ; d < dim ; d+=1) {
                if (index.at(d) > 0) {
                    auto i = index; i.at(d)-=1;
                    neighbours.template emplace_back(i);
                }
                if (index.at(d) < (gridSize_.at(d)-1)) {
                    auto i = index; i.at(d)+=1;
                    neighbours.template emplace_back(i);
                }
            }
            return neighbours;
        }

        std::vector<std::array<indexDType, dim>> allNeighbourCellIndexes(const std::array<indexDType, dim>& index) {

            constexpr auto max_neighbours = static_cast<size_t>(std::pow(3,dim)-1);
            auto neighbourIndexOffsets = getNeighbourIndexOffsets<max_neighbours>();
            auto neighbours = getValidNeighbourIndexes<max_neighbours>(index, neighbourIndexOffsets);
            return neighbours;
        }

        const std::array<indexDType, dim>& getGridSize() const {return gridSize_;};

        const std::array<distanceDType, dim>& getCellSize() const {return cellSize_;};

        void clear(CellType clearValue) {
            for (size_t i = 0 ; i < memorySize ; i++) {occGrid_[i] = clearValue;}
        }

    protected:

        const std::array<size_t, sizeof...(gridSize)> gridSize_ = {gridSize...};
        const std::array<distanceDType, dim> cellSize_;
        std::array<CellType, memorySize> occGrid_;

        indexDType indexTo1D(const std::array<indexDType, dim>& index) const {
            indexDType index1D = 0;
            indexDType cumulativeDimSize = 1;
            for (size_t i = 0 ; i < index.size() ; i+=1) {
                index1D += index.at(i) * cumulativeDimSize;
                cumulativeDimSize *= gridSize_.at(i);
            }
            return index1D;
        }

        template<size_t max_size>
        std::array<std::array<int, dim>, max_size> getNeighbourIndexOffsets() {

            std::array<int, 3> index_offsets = {0, -1, 1};
            std::array<indexDType, dim> period_helper;
            for (size_t d = 0 ; d < dim ; d+=1) {
                period_helper.at(d) = std::pow(3,d);
            }

            std::array<std::array<int, dim>, max_size> offsets = {0};
            for (size_t d = 0 ; d < dim ; d+=1) {
                size_t counter = 1;
                size_t index_offset_pointer = 0;
                for (size_t i = 0 ; i < max_size+1 ; i+=1) {
                    if (i > 0) { offsets.at(i - 1).at(d) = index_offsets.at(index_offset_pointer); }
                    if (counter == period_helper.at(d)) {
                        counter = 0;
                        index_offset_pointer = (index_offset_pointer+1) % 3;
                    }
                    counter+=1;
                }
            }
            return offsets;
        }

        template<size_t max_size>
        std::vector<std::array<indexDType, dim>> getValidNeighbourIndexes(const std::array<indexDType, dim>& baseIndex,
                                                                          const std::array<std::array<int, dim>, max_size> offsetArray) {
            std::vector<std::array<indexDType, dim>> neighbours = {};
            neighbours.reserve(max_size);
            for (const auto& offset : offsetArray) {
                bool validOffset = true;
                for (size_t d = 0 ; d < dim ; d+=1) {
                    int tmp = offset.at(d) + baseIndex.at(d);
                    if (tmp < 0 || tmp >= static_cast<int>(gridSize_.at(d))) { validOffset = false; break;}
                }
                if (validOffset) {
                    std::array<indexDType, dim> new_index = {0};
                    for (size_t d = 0 ; d < dim ; d+=1) { new_index.at(d) = baseIndex.at(d) + offset.at(d); }
                    neighbours.emplace_back(new_index);
                }
            }
            return neighbours;
        }

        [[nodiscard]] bool indexIsValid(const std::array<indexDType, 2>& index) const {
            for (size_t d = 0 ; d < 2 ; d+=1) {
                if (index.at(d) > (gridSize_.at(d)-1)) {
                    return false;
                }
            }
            return true;
        }
    };

    template<size_t dim, typename CellType, typename distanceDType, size_t... gridSize>
    class OccupancyMapND : public OccupancyMapND_common<dim, CellType, distanceDType, gridSize...> {
    public:
        OccupancyMapND() = delete;
        explicit OccupancyMapND(const std::array<distanceDType, dim>& cellSize)
                : OccupancyMapND_common<dim, CellType, distanceDType, gridSize...> {cellSize} {

        }
    };

    template<typename CellType, typename distanceDType, size_t... gridSize>
    class OccupancyMapND<2, CellType, distanceDType, gridSize...> : public OccupancyMapND_common<2, CellType, distanceDType, gridSize...>  {
    public:
        enum class Direction {
            X_UP,
            X_DOWN,
            Y_UP,
            Y_DOWN,
        };

        OccupancyMapND() = delete;
        explicit OccupancyMapND(const std::array<distanceDType, 2>& cellSize)
                : OccupancyMapND_common<2, CellType, distanceDType, gridSize...> {cellSize}{

        }

        std::optional<std::array<indexDType, 2>> neighbourInDirection(std::array<indexDType, 2> index, const Direction& dir) {
            std::array<indexDType, 2> offset = index;
            if (dir == Direction::X_UP) {offset.at(0) += 1;}
            if (dir == Direction::X_DOWN) {offset.at(0) -= 1;}
            if (dir == Direction::Y_UP) {offset.at(1) += 1;}
            if (dir == Direction::Y_DOWN) {offset.at(1) -= 1;}
            if (this->indexIsValid(offset)) {
                return offset;
            }
            return std::nullopt;
        }
    };

    template<typename CellType, typename distanceDType, size_t... gridSize>
    class OccupancyMapND<3, CellType, distanceDType, gridSize...> : public OccupancyMapND_common<3, CellType, distanceDType, gridSize...>  {
    public:
        enum class Direction {
            X_UP,
            X_DOWN,
            Y_UP,
            Y_DOWN,
            Z_UP,
            Z_DOWN,
        };

        OccupancyMapND() = delete;
        explicit OccupancyMapND(const std::array<distanceDType, 3>& cellSize)
                : OccupancyMapND_common<3, CellType, distanceDType, gridSize...> {cellSize}{

        }

        std::optional<std::array<indexDType, 3>> neighbourInDirection(std::array<indexDType, 3> index, const Direction& dir) {
            std::array<indexDType, 3> offset = index;
            if (dir == Direction::X_UP) {offset.at(0) += 1;}
            if (dir == Direction::X_DOWN) {offset.at(0) -= 1;}
            if (dir == Direction::Y_UP) {offset.at(1) += 1;}
            if (dir == Direction::Y_DOWN) {offset.at(1) -= 1;}
            if (dir == Direction::Z_UP) {offset.at(2) += 1;}
            if (dir == Direction::Z_DOWN) {offset.at(2) -= 1;}
            if (this->indexIsValid(offset)) {
                return offset;
            }
            return std::nullopt;
        }
    };

}

#endif //ROBOTICTEMPLATELIBRARY_OCCUPANCYMAPND_H
