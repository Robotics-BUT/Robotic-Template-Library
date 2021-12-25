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


#ifndef ROBOTICTEMPLATELIBRARY_MUNKRES_H
#define ROBOTICTEMPLATELIBRARY_MUNKRES_H

#include <array>
#include <algorithm>

namespace rtl
{

    /*!
     * Implementation of the Munkres (Hungarian) algorithm. Also called assignment algorithm. Takes cost matrix at the
     * input (columns - workers, rows - tasks) and search for the combination with the smallest cost sum.
     *
     * Implementation based on: https://brc2.com/the-algorithm-workshop/
     *
     * Algorithm is also suitable for pairing detected ojects between two frames (maximize Intersecion over Union of the
     * bounding boxes).
     *
     * @tparam T Data type of values in the const matrix
     * @tparam N Dimension of the cost matrix (NxN)
     * */
    template <typename T, size_t N>
    class Munkres {

    enum class Step {
        STEP_ONE,
        STEP_TWO,
        STEP_THREE,
        STEP_FOUR,
        STEP_FIVE,
        STEP_SIX,
        STEP_SEVEN
    };

    public:

        struct Result {
            Result() : col{0}, row{0} {}
            Result(size_t c, size_t r, T cst) : col{c}, row{r}, cost{cst} {};
            size_t col;
            size_t row;
            T cost;
        };

        /*!
         * Solves assignment problem for given cost matrix.
         *
         * @param cost_matrix cost matrix of a given problem (cols: workers, rows: tasks). Matrix must be square-shaped
         * @param max_cost If true, algorithm maximize sum of all costs (sitable for best IoU search)
         * */
        static std::array<Result, N> solve(Matrix<N, N, T> cost_matrix, bool max_cost = false) {

            Step step = Step::STEP_ONE;
            VectorND<N, bool> row_cover{};
            VectorND<N, bool> col_cover{};
            Matrix<N, N, uint8_t> mask{};
            std::optional<std::pair<size_t, size_t>> z0_row_col{};
            Matrix<N, N, T> cost_matrix_backup = cost_matrix;

            if (max_cost) {
                flip_costs(cost_matrix);
            }

            while(true) {
                switch (step) {
                    case Step::STEP_ONE:
                        step_one(cost_matrix, step);
                        break;
                    case Step::STEP_TWO:
                        step_two(cost_matrix, mask, step);
                        break;
                    case Step::STEP_THREE:
                        step_three(mask, col_cover, step);
                        break;
                    case Step::STEP_FOUR:
                        z0_row_col = step_four(cost_matrix, mask, row_cover, col_cover, step);
                        break;
                    case Step::STEP_FIVE:
                        step_five(mask, row_cover, col_cover, z0_row_col.value(), step);
                        break;
                    case Step::STEP_SIX:
                        step_six(cost_matrix, row_cover, col_cover, step);
                        break;
                    case Step::STEP_SEVEN:
                        return step_seven(cost_matrix_backup, mask);
                }
            }
        }

    protected:

        static void flip_costs(Matrix<N, N, T>& cost_matrix) {
            auto max = std::numeric_limits<T>::min();
            for (int r = 0 ; r < cost_matrix.rowNr() ; r++) {
                for (int c = 0 ; c < cost_matrix.colNr() ; c++) {
                    if (cost_matrix.getElement(r, c) > max) { max = cost_matrix.getElement(r, c); }
                }
            }

            for (int r = 0 ; r < cost_matrix.rowNr() ; r++) {
                for (int c = 0 ; c < cost_matrix.colNr() ; c++) {
                    cost_matrix.setElement(r, c, -(cost_matrix.getElement(r,c) - max) );
                }
            }
        }


        static void step_one(Matrix<N, N, T>& cost_matrix, Step& step) {
            for (int r = 0 ; r < cost_matrix.rowNr() ; r++) {
                T row_min = std::numeric_limits<T>::max();

                for (int c = 0 ; c < cost_matrix.colNr() ; c++) {
                    if (cost_matrix.getElement(r, c) < row_min) {row_min = cost_matrix.getElement(r, c);}
                }

                for (int c = 0 ; c < cost_matrix.colNr() ; c++) {
                    cost_matrix.setElement(r, c, cost_matrix.getElement(r, c) - row_min);
                }
            }
            step = Step::STEP_TWO;
        }


        static void step_two(Matrix<N, N, T>& cost_matrix,
                             Matrix<N, N, uint8_t>& mask,
                             Step& step) {

            auto row_cover = VectorND<N, bool>::zeros();
            auto col_cover = VectorND<N, bool>::zeros();

            for (int r = 0 ; r < cost_matrix.rowNr() ; r++) {
                for (int c = 0 ; c < cost_matrix.colNr() ; c++) {

                    if (cost_matrix.getElement(r, c) == 0 && row_cover[r] == 0 && col_cover[c] == 0) {
                        mask.setElement(r, c, 1);
                        row_cover[r] = 1;
                        col_cover[c] = 1;
                    }
                }
            }
            step = Step::STEP_THREE;
        }


        static void step_three(const Matrix<N, N, uint8_t>& mask,
                               VectorND<N, bool>& col_cover,
                               Step& step) {

            size_t covered_col_count = 0;
            for (int r = 0 ; r < mask.rowNr() ; r++) {
                for (int c = 0 ; c < mask.colNr() ; c++) {
                    if (mask.getElement(r, c) == 1) {
                        col_cover[c] = 1;
                        covered_col_count += 1;
                    }
                }
            }

            if (covered_col_count >= N) {
                step = Step::STEP_SEVEN;
            } else {
                step = Step::STEP_FOUR;
            }
        }


        static std::optional<std::pair<size_t, size_t>> step_four(Matrix<N, N, T>& cost_matrix,
                                                                  Matrix<N, N, uint8_t>& mask,
                                                                  VectorND<N, bool>& row_cover,
                                                                  VectorND<N, bool>& col_cover,
                                                                  Step& step) {

            while (true) {

                auto row_col = find_uncovered_zero(cost_matrix, row_cover, col_cover);

                if (row_col == std::nullopt) {
                    step = Step::STEP_SIX;
                    return std::nullopt;
                } else {
                    mask.setElement(row_col->first, row_col->second, 2);
                    row_col = star_in_row(mask, row_col->first);

                    if (row_col != std::nullopt) {
                        row_cover.setElement(row_col->first, 1);
                        col_cover.setElement(row_col->second, 0);
                    } else {
                        step = Step::STEP_FIVE;
                        return std::make_optional<std::pair<size_t, size_t>>(row_col->first, row_col->second);
                    }
                }
            }
        }


        static void step_five(Matrix<N, N, uint8_t>& mask,
                              VectorND<N, bool>& row_cover,
                              VectorND<N, bool>& col_cover,
                              std::pair<size_t, size_t>& z0_row_col,
                              Step& step) {

            auto path_count = 1;
            Matrix<N*2, 2, size_t> path;

            path.setElement(path_count - 1, 0, z0_row_col.first);
            path.setElement(path_count - 1, 1, z0_row_col.second);
            bool ready = false;

            while(!ready) {
                auto row_col = star_in_col(mask, path.getElement(path_count - 1, 1));
                if (row_col != std::nullopt) {
                    path_count += 1;
                    path.setElement(path_count - 1, 0, row_col->first);
                    path.setElement(path_count - 1, 1, path.getElement(path_count - 2, 1));
                } else {
                    ready = true;
                }
                if (!ready) {
                    row_col = prime_in_row(mask, path.getElement(path_count - 1, 0));
                    path_count += 1;
                    path.setElement(path_count - 1, 0, path.getElement(path_count - 2, 0));
                    path.setElement(path_count - 1, 1, row_col->second);
                }
            }

            augment_path(mask, path, path_count);
            clear_covers(row_cover, col_cover);
            erase_primes(mask);
            step = Step::STEP_THREE;
        }


        static void step_six(Matrix<N, N, T>& cost_matrix,
                             VectorND<N, bool>& row_cover,
                             VectorND<N, bool>& col_cover,
                             Step& step) {

            auto min_value = minimal_value(cost_matrix, row_cover, col_cover);
            for (int r = 0 ; r < cost_matrix.rowNr() ; r++) {
                for (int c = 0 ; c < cost_matrix.colNr() ; c++) {
                    if (row_cover.getElement(r) == 1) {
                        cost_matrix.setElement(r, c, cost_matrix.getElement(r, c) + min_value);
                    }
                    if (col_cover.getElement(c) == 0) {
                        cost_matrix.setElement(r, c, cost_matrix.getElement(r, c) - min_value);
                    }
                }
            }
            step = Step::STEP_FOUR;
        }


        static std::array<Result, N> step_seven(const Matrix<N, N, T>& cost_matrix,
                                                const Matrix<N, N, uint8_t>& mask) {
            std::array<Result, N> output;
            for (int r = 0 ; r < cost_matrix.rowNr() ; r++) {
                auto row_col = star_in_row(mask, r);
                output.at(r) = Result(row_col->first, row_col->second, cost_matrix.getElement(r, row_col->second));
            }
            return output;
        }


        static std::optional<std::pair<size_t, size_t>> find_uncovered_zero(const Matrix<N, N, T>& cost_matrix,
                                                                            const VectorND<N, bool>& row_cover,
                                                                            const VectorND<N, bool>& col_cover) {

            for (int r = 0 ; r < cost_matrix.rowNr() ; r++) {
                for (int c = 0 ; c < cost_matrix.colNr() ; c++) {
                    if (cost_matrix.getElement(r, c) == 0 && row_cover.getElement(r) == 0 && col_cover.getElement(c) == 0) {
                        return std::make_optional<std::pair<size_t, size_t>>(r, c);
                    }
                }
            }

            return std::nullopt;
        }


        static std::optional<std::pair<size_t, size_t>> star_in_row(const Matrix<N, N, uint8_t>& mask, size_t row) {
            for (int c = 0 ; c < mask.colNr() ; c++) {
                if (mask.getElement(row, c) == 1) {
                    return std::make_optional<std::pair<size_t, size_t>>(row, c);
                }
            }
            return std::nullopt;
        }


        static std::optional<std::pair<size_t, size_t>> star_in_col(const Matrix<N, N, uint8_t>& mask,
                                                                    size_t col) {

            for (int r = 0 ; r < mask.rowNr() ; r++) {
                if (mask.getElement(r, col) == 1) {
                    return std::make_optional<std::pair<size_t, size_t>>(r, col);
                }
            }
            return std::nullopt;
        };


        static std::optional<std::pair<size_t, size_t>> prime_in_row(const Matrix<N, N, uint8_t>& mask,
                                                                     size_t row) {

            for (int c = 0 ; c < mask.colNr() ; c++) {
                if (mask.getElement(row, c) == 2) {
                    return std::make_optional<std::pair<size_t, size_t>>(row, c);
                }
            }
            return std::nullopt;
        }


        static void augment_path(Matrix<N, N, uint8_t>& mask,
                                 Matrix<N*2, 2, size_t>& path,
                                 size_t path_count) {

            for (size_t p = 0 ; p < path_count ; p++) {
                if (mask.getElement(path.getElement(p,0),path.getElement(p,1)) == 1) {
                    mask.setElement(path.getElement(p,0),path.getElement(p,1), 0);
                }
                else {
                    mask.setElement(path.getElement(p,0),path.getElement(p,1), 1);
                }
            }
        }


        static void clear_covers(VectorND<N, bool>& row_cover,
                                 VectorND<N, bool>& col_cover) {
            for (int i = 0 ; i < row_cover.length() ; i++) { row_cover.setElement(i, 0);}
            for (int i = 0 ; i < col_cover.length() ; i++) { col_cover.setElement(i, 0);}
        }


        static void erase_primes(Matrix<N, N, uint8_t>& mask) {
            for (int r = 0 ; r < mask.rowNr() ; r++) {
                for (int c = 0 ; c < mask.colNr() ; c++) {
                    if (mask.getElement(r, c) == 2) {mask.setElement(r, c, 0);}
                }
            }
        }


        static T minimal_value(const Matrix<N, N, T>& cost_matrix,
                               const VectorND<N, bool>& row_cover,
                               const VectorND<N, bool>& col_cover) {

            auto min_value = std::numeric_limits<T>::max();
            for (int r = 0 ; r < cost_matrix.rowNr() ; r++) {
                for (int c = 0 ; c < cost_matrix.colNr() ; c++) {
                    if (row_cover.getElement(r) == 0 && col_cover.getElement(c) == 0){
                        if (min_value > cost_matrix.getElement(r, c)) {
                            min_value = cost_matrix.getElement(r, c);
                        }
                    }
                }
            }
            return min_value;
        }
    };
}

#endif //ROBOTICTEMPLATELIBRARY_MUNKRES_H