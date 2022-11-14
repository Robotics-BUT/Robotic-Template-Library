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

#ifndef ROBOTICTEMPLATELIBRARY_UNSTABLE_H
#define ROBOTICTEMPLATELIBRARY_UNSTABLE_H

#include "rtl/unstable/AStarCell.h"
#include "rtl/unstable/Dijkstra.h"
#include "rtl/unstable/DijkstraCell.h"
#include "rtl/unstable/OccupancyMapND.h"
#include "rtl/unstable/OccupancyMapPathFinder.h"
#include "rtl/unstable/PathFinderNode.h"

namespace rtl {
    template<typename Element>
    using Occupancy2D = OccupancyMapND<2, Element>;                 //!< Partial OccupancyMap specialization for two dimensions
    using OccupancyMap2Df = OccupancyMapND<2, float>;                  //!< Full OccupancyMap specialization for two dimensions and float elements.
    using OccupancyMap2Dd = OccupancyMapND<2, double>;                 //!< Full OccupancyMap specialization for two dimensions and double elements.

    template<typename Element>
    using OccupancyMap3D = OccupancyMapND<3, Element>;                 //!< Partial OccupancyMap specialization for three dimensions
    using OccupancyMap3Df = OccupancyMapND<3, float>;                  //!< Full OccupancyMap specialization for three dimensions and float elements.
    using OccupancyMap3Dd = OccupancyMapND<3, double>;                 //!< Full OccupancyMap specialization for three dimensions and double elements.
}

#endif // ROBOTICTEMPLATELIBRARY_UNSTABLE_H
