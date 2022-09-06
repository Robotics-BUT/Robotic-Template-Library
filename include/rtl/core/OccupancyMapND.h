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
    template<unsigned int dim, typename CellType>
    class OccupancyMapND {

    public:
        OccupancyMapND(const std::array<size_t, dim>& size)
                : size_{size} {
            static_assert(dim == 0, "Occupancy Map has to be non-zero dimension");
            constexpr size_t arraySize = std::accumulate(size, size.size(), 0);
            occGrid_ = new CellType[arraySize];
        }

        ~OccupancyMapND() {
            delete occGrid_;
        }

        std::optional<const CellType&> getCell(const std::array<size_t, dim>& index) const {
            if (!indexIsValid(index)) {
                return std::nullopt;
            }
            return std::optional<const CellType&>{occGrid_[convertNDimIndex(index)]};
        }

        void setCell(const CellType& cell, const std::array<size_t, dim>& index) {
            if (!indexIsValid(index)) {
                return;
            }
            *occGrid_[convertNDimIndex(index)] = cell;
        }

    private:

        const std::array<size_t, dim>& size_;
        CellType* occGrid_;

        bool indexIsValid(const std::array<size_t, dim>& index) const {
            for(size_t i = 0 ; i < dim ; i++) {
                if (index.at(i) >= size_.at(i)) {return false;}
            }
            return true;
        }

        size_t convertNDimIndex(const std::array<size_t, dim>& index) const {
            size_t oneDimIndex = 0;
            size_t cumulativeDimSize = 1;
            for (const auto& i : index) {
                oneDimIndex += i * cumulativeDimSize;
                cumulativeDimSize *= size_.at(i);
            }
            return oneDimIndex;
        }
    };
}

#endif //ROBOTICTEMPLATELIBRARY_OCCUPANCYMAPND_H
