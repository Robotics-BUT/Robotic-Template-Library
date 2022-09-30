// This file is part of the Robotic Template Library (RTL), a C++
// template library for usage in robotic research and applications
// under the MIT licence:
//
// Copyright 2021 Brno University of Technology
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

#ifndef ROBOTICTEMPLATELIBRARY_ASTARCELL_H
#define ROBOTICTEMPLATELIBRARY_ASTARCELL_H

namespace rtl
{
    template<typename costDType = float>
    class AStarCell {
    public:
        [[nodiscard]] costDType getGCost() const {return g_cost_;}
        [[nodiscard]] costDType getHCost() const {return h_cost_;}
        [[nodiscard]] costDType getFCost() const {return g_cost_ + h_cost_;}
        void setGCost(costDType g_cost) {g_cost_ = g_cost;}
        void setHCost(costDType h_cost) {h_cost_ = h_cost;}
    private:
        costDType g_cost_;
        costDType h_cost_;
    };
}

#endif //ROBOTICTEMPLATELIBRARY_ASTARCELL_H
