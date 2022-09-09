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
    template<unsigned int dim, typename CellType, typename distanceDType = float>
    class OccupancyMapND {

        using indexDType = int;

        constexpr indexDType mult(indexDType const &a, indexDType const &b) { return a * b; }

        template <size_t... Is, size_t N>
        constexpr std::array<indexDType, N> multiply(std::array<indexDType, N> const &src,
                                            std::index_sequence<Is...>, indexDType const &mul) {
            return std::array<indexDType, N>{{mult(src[Is], mul)...}};
        }

    public:
        OccupancyMapND(const std::array<indexDType, dim>& gridSize, const std::array<distanceDType, dim>& cellSize, const std::array<indexDType, dim>& indexOffset = {0})
                : gridSize_{gridSize}
                , cellSize_{cellSize}
                , indexOffset_{indexOffset}{
            static_assert(dim != 0, "Occupancy Map has to be non-zero dimension");
            indexDType arraySize = 1;
            std::for_each(gridSize.begin(), gridSize.end(), [&](auto value){
                arraySize *= value;
            });
            occGrid_ = new CellType[arraySize];
        }

        ~OccupancyMapND() {
            delete occGrid_;
        }

        [[nodiscard]] std::optional<const CellType&> getCell(const std::array<indexDType, dim>& index) const {
            if (!indexIsValid(index)) {
                return std::nullopt;
            }
            return std::optional<const CellType&>{occGrid_[indexTo1D(index)]};
        }

        void setCell(const CellType& cell, const std::array<indexDType, dim>& index) {
            if (!indexIsValid(index)) {
                return;
            }
            *occGrid_[indexTo1D(index)] = cell;
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
                outputIndex.at(i) = static_cast<indexDType>(coordinates.at(i) % cellSize_.at(i));
            }
            return outputIndex;
        }

        [[nodiscard]] std::array<distanceDType, dim> distance(const std::array<indexDType, dim>& i1, const std::array<indexDType, dim>& i2) {
            std::array<distanceDType, dim> outputDistance;
            for(size_t i = 0 ; i < dim ; i++) {
                outputDistance.at(i) = cellSize_.at(i) * (i2.at(i) - i1.at(i));
            }
            return outputDistance;
        }

    private:

        const std::array<indexDType, dim>& gridSize_;
        const std::array<distanceDType, dim>& cellSize_;
        const std::array<indexDType, dim>& indexOffset_;
        CellType* occGrid_;

        bool indexIsValid(const std::array<indexDType, dim>& index) const {
            for(indexDType i = 0 ; i < dim ; i++) {
                if (index.at(i) >= gridSize_.at(i)) {return false;}
            }
            return true;
        }

        indexDType indexTo1D(const std::array<indexDType, dim>& index) const {
            indexDType index1D = 0;
            indexDType cumulativeDimSize = 1;
            for (const auto& i : index) {
                index1D += i * cumulativeDimSize;
                cumulativeDimSize *= gridSize_.at(i);
            }
            return index1D;
        }


    };
}

#endif //ROBOTICTEMPLATELIBRARY_OCCUPANCYMAPND_H
