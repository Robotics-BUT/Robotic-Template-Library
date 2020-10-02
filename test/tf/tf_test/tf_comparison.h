#pragma once

template <int dim, typename dtype>
bool CompareTfsEqual(const rtl::RigidTfND <dim, dtype> &tf1, const rtl::RigidTfND <dim, dtype> &tf2) {

    auto vd = rtl::VectorND<dim, dtype>::distance( tf1.trVec(), tf2.trVec());
    if (vd >= rtl::test::type<rtl::VectorND<dim, dtype>>::allowedError())
        return false;

    auto matd = rtl::Matrix<dim, dim, dtype>::distance( tf1.rotMat(), tf2.rotMat());
    if (matd >= rtl::test::type<rtl::Matrix<dim, dim, dtype>>::allowedError())
        return false;

    return true;
}


template <int dim, typename dtype>
bool CompareTransEqual(const rtl::TranslationND <dim, dtype> &tf1, const rtl::TranslationND <dim, dtype> &tf2) {

    auto vd = rtl::VectorND<dim, dtype>::distance( tf1.trVec(), tf2.trVec());
    if (vd >= rtl::test::type<rtl::VectorND<dim, dtype>>::allowedError())
        return false;

    return true;
}


template <int dim, typename dtype>
bool CompareRotsEqual(const rtl::RotationND <dim, dtype> &tf1, const rtl::RotationND <dim, dtype> &tf2) {

    auto matd = rtl::Matrix<dim, dim, dtype>::distance( tf1.rotMat(), tf2.rotMat());
    if (matd >= rtl::test::type<rtl::Matrix<dim, dim, dtype>>::allowedError())
        return false;

    return true;
}




template <int dim, typename dtype>
bool CompareTfsNotEqual(const rtl::RigidTfND<dim, dtype>& tf1, const rtl::RigidTfND<dim, dtype>& tf2) {

    auto vd = rtl::VectorND<dim, dtype>::distance( tf1.trVec(), tf2.trVec());
    if (vd <= rtl::test::type<rtl::VectorND<dim, dtype>>::allowedError())
        return false;

    auto matd = rtl::Matrix<dim, dim, dtype>::distance( tf1.rotMat(), tf2.rotMat());
    if (matd <= rtl::test::type<rtl::Matrix<dim, dim, dtype>>::allowedError())
        return false;

    return true;
}

template <int dim, typename dtype>
bool CompareTransNotEqual(const rtl::TranslationND<dim, dtype>& tf1, const rtl::TranslationND<dim, dtype>& tf2) {

    auto vd = rtl::VectorND<dim, dtype>::distance( tf1.trVec(), tf2.trVec());
    if (vd <= rtl::test::type<rtl::VectorND<dim, dtype>>::allowedError())
        return false;

    return true;
}


template <int dim, typename dtype>
bool CompareRotsNotEqual(const rtl::RotationND<dim, dtype>& tf1, const rtl::RotationND<dim, dtype>& tf2) {

    auto matd = rtl::Matrix<dim, dim, dtype>::distance( tf1.rotMat(), tf2.rotMat());
    if (matd <= rtl::test::type<rtl::Matrix<dim, dim, dtype>>::allowedError())
        return false;

    return true;
}