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

namespace rtl
{
    template<size_t dim, typename CellType, typename distanceDType = float>
    class OccupancyMapND {

        using indexDType = size_t;

        template<typename T, size_t N>
        static constexpr size_t array_product(std::array<T, N> arr) {
            T prod = 1;
            for (size_t i = 0 ; i < N ; i++) {prod *= arr.at(i);}
            return prod;
        };

    public:
        OccupancyMapND(const std::array<indexDType, dim>& gridSize, const std::array<distanceDType, dim>& cellSize)
                : gridSize_{gridSize}
                , cellSize_{cellSize}{
            static_assert(dim != 0, "Occupancy Map has to be non-zero dimension");
            occGrid_ = new CellType[array_product(gridSize_)];
        }

        ~OccupancyMapND() {
            delete occGrid_;
        }

        [[nodiscard]] const CellType& getCell(const std::array<indexDType, dim>& index) const {
            auto i = indexTo1D(index);
            return occGrid_[i];
        }

        void setCell(const CellType& cell, const std::array<indexDType, dim>& index) {
            auto i = indexTo1D(index);
            occGrid_[i] = cell;
        }

        [[nodiscard]] std::array<distanceDType, dim> indexToCoordinates(const std::array<indexDType, dim>& index) const {
            std::array<distanceDType, dim> outputCoordinates;
            for(size_t i = 0 ; i < dim ; i++) {
                outputCoordinates.at(i) = cellSize_.at(i) * (index.at(i) + 0.5);
            }
            return outputCoordinates;
        }

        [[nodiscard]] std::array<indexDType, dim> coordinatesToIndex(const std::array<distanceDType, dim>& coordinates) const {
            std::array<indexDType, dim> outputIndex;
            for(size_t i = 0 ; i < dim ; i++) {
                outputIndex.at(i) = static_cast<indexDType>(std::floor(coordinates.at(i) / cellSize_.at(i)));
            }
            return outputIndex;
        }

        distanceDType euclidean_distance(const std::array<indexDType, dim>& i1, const std::array<indexDType, dim>& i2) const {
            auto dist = distanceInAxis(i1, i2);
            distanceDType sum = 0;
            for (const auto& d : dist) {
                sum += std::pow(d, 2);
            }
            return std::sqrt(sum);
        }

        [[nodiscard]] std::array<distanceDType, dim> distanceInAxis(const std::array<indexDType, dim>& i1, const std::array<indexDType, dim>& i2) const {
            std::array<distanceDType, dim> outputDistance;
            for(size_t i = 0 ; i < dim ; i++) {
                outputDistance.at(i) = cellSize_.at(i) * (i2.at(i) - i1.at(i));
            }
            return outputDistance;
        }

    private:

        const std::array<indexDType, dim> gridSize_;
        const std::array<distanceDType, dim> cellSize_;
        CellType* occGrid_;

        indexDType indexTo1D(const std::array<indexDType, dim>& index) const {
            indexDType index1D = 0;
            indexDType cumulativeDimSize = 1;
            for (size_t i = 0 ; i < index.size() ; i++) {
                index1D += index.at(i) * cumulativeDimSize;
                cumulativeDimSize *= gridSize_.at(i);
            }
            return index1D;
        }
    };
}

#endif //ROBOTICTEMPLATELIBRARY_OCCUPANCYMAPND_H
