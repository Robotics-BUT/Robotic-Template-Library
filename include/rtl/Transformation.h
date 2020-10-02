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
// Contact person: Ales Jelinek <Ales.Jelinek@ceitec.vutbr.cz>

#ifndef ROBOTICTEMPLATELIBRARY_TRANSFORMATION_H
#define ROBOTICTEMPLATELIBRARY_TRANSFORMATION_H

#include "rtl/tf/TranslationND.h"
#include "rtl/tf/RotationND.h"
#include "rtl/tf/RigidTfND.h"
#include "rtl/tf/TfTree.h"
#include "rtl/tf/TfChain.h"

namespace rtl
{
    template<typename T>
    using Translation2D = TranslationND<2, T>;
    using Translation2f = Translation2D<float>;
    using Translation2d = Translation2D<double>;

    template<typename T>
    using Translation3D = TranslationND<3, T>;
    using Translation3f = Translation3D<float>;
    using Translation3d = Translation3D<double>;

    template<typename T>
    using Rotation2D = RotationND<2, T>;
    using Rotation2f = Rotation2D<float>;
    using Rotation2d = Rotation2D<double>;

    template<typename T>
    using Rotation3D = RotationND<3, T>;
    using Rotation3f = Rotation3D<float>;
    using Rotation3d = Rotation3D<double>;

    template<typename T>
    using RigidTf2D = RigidTfND<2, T>;
    using RigidTf2f = RigidTf2D<float>;
    using RigidTf2d = RigidTf2D<double>;

    template<typename T>
    using RigidTf3D = RigidTfND<3, T>;
    using RigidTf3f = RigidTf3D<float>;
    using RigidTf3d = RigidTf3D<double>;
}

#endif //ROBOTICTEMPLATELIBRARY_TRANSFORMATION_H
