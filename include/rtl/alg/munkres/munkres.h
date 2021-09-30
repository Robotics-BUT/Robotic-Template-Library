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

// Implementation based on: https://brc2.com/the-algorithm-workshop/

#ifndef ROBOTICTEMPLATELIBRARY_MUNKRES_H
#define ROBOTICTEMPLATELIBRARY_MUNKRES_H

#include <array>
#include <algorithm>

namespace rtl
{
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
            Result() : worker{0}, job{0} {}
            Result(size_t w, size_t j, T c) : worker{w}, job{j}, cost{c} {};
            size_t worker;
            size_t job;
            T cost;
        };

        static std::array<Result, N> solve(std::array<std::array<T, N>, N> cost_matrix, bool max_cost = false) {

            Step step = Step::STEP_ONE;
            std::array<bool, N> row_cover{};
            std::array<bool, N> col_cover{};
            std::array<std::array<uint8_t, N>, N> mask{};
            std::optional<std::pair<size_t, size_t>> z0_row_col{};
            std::array<std::array<T, N>, N> cost_matrix_backup = cost_matrix;

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

        static void flip_costs(std::array<std::array<T, N>, N>& cost_matrix) {
            auto max = std::numeric_limits<T>::min();
            for (const auto& row : cost_matrix) {
                auto row_max = *std::max_element(row.begin(), row.end());
                if (row_max > max) {max = row_max;}
            }

            for (auto& row : cost_matrix) {
                for (auto& element : row) {
                    element = -(element - max);
                }
            }
        }


        static void step_one(std::array<std::array<T, N>, N>& cost_matrix, Step& step) {
            for (auto& row : cost_matrix) {
                T row_minimum = *std::min_element(row.begin(), row.end());
                std::for_each(row.begin(), row.end(), [row_minimum](T& element){element -= row_minimum;});
            }
            step = Step::STEP_TWO;
        }


        static void step_two(std::array<std::array<T, N>, N>& cost_matrix,
                             std::array<std::array<uint8_t , N>, N>& mask,
                             Step& step) {

            std::array<bool, N> row_cover{};
            std::array<bool, N> col_cover{};

            {size_t r = 0; for (auto& row : cost_matrix) {
                {size_t c = 0; for (auto& element : row) {
                    if (element == 0 && row_cover[r] == 0 && col_cover[c] == 0) {
                        mask[r][c] = 1;
                        row_cover[r] = 1;
                        col_cover[c] = 1;
                    }
                    c += 1;
                }}
                r += 1;
            }}
            step = Step::STEP_THREE;
        }


        static void step_three(const std::array<std::array<uint8_t, N>, N>& mask,
                               std::array<bool, N>& col_cover,
                               Step& step) {

            size_t covered_col_count = 0;
            {size_t r = 0; for (auto& mask_row : mask) {
                    {size_t c = 0; for (auto& mask_element : mask_row) {
                        if (mask_element == 1) {
                            col_cover[c] = 1;
                            covered_col_count += 1;
                        }
                        c += 1;
                    }}
                    r += 1;
            }}

            if (covered_col_count >= N) {
                step = Step::STEP_SEVEN;
            } else {
                step = Step::STEP_FOUR;
            }
        }


        static std::optional<std::pair<size_t, size_t>> step_four(std::array<std::array<T, N>, N>& cost_matrix,
                              std::array<std::array<uint8_t, N>, N>& mask,
                              std::array<bool, N>& row_cover,
                              std::array<bool, N>& col_cover,
                              Step& step) {

            while (true) {

                auto row_col = find_uncovered_zero(cost_matrix, row_cover, col_cover);

                if (row_col == std::nullopt) {
                    step = Step::STEP_SIX;
                    return std::nullopt;
                } else {
                    mask[row_col->first][row_col->second] = 2;
                    row_col = star_in_row(mask, row_col->first);

                    if (row_col != std::nullopt) {
                        row_cover.at(row_col->first) = 1;
                        col_cover.at(row_col->second) = 0;
                    } else {
                        step = Step::STEP_FIVE;
                        return std::make_optional<std::pair<size_t, size_t>>(row_col->first, row_col->second);
                    }
                }
            }
        }


        static void step_five(std::array<std::array<uint8_t, N>, N>& mask,
                              std::array<bool, N>& row_cover,
                              std::array<bool, N>& col_cover,
                              std::pair<size_t, size_t>& z0_row_col,
                              Step& step) {

            auto path_count = 1;
            std::array<std::array<size_t, 2>, N*2> path;

            path[path_count - 1][0] = z0_row_col.first;
            path[path_count - 1][1] = z0_row_col.second;
            bool ready = false;

            while(!ready) {
                auto row_col = star_in_col(mask, path[path_count - 1][1]);
                if (row_col != std::nullopt) {
                    path_count += 1;
                    path[path_count - 1][0] = row_col->first;
                    path[path_count - 1][1] = path[path_count - 2][1];
                } else {
                    ready = true;
                }
                if (!ready) {
                    row_col = prime_in_row(mask, path[path_count - 1][0]);
                    path_count += 1;
                    path[path_count - 1][0] = path[path_count - 2][0];
                    path[path_count - 1][1] = row_col->second;
                }
            }

            augment_path(mask, path, path_count);
            clear_covers(row_cover, col_cover);
            erase_primes(mask);
            step = Step::STEP_THREE;
        }


        static void step_six(std::array<std::array<T, N>, N>& cost_matrix,
                             std::array<bool, N>& row_cover,
                             std::array<bool, N>& col_cover,
                             Step& step) {

            auto min_value = minimal_value(cost_matrix, row_cover, col_cover);
            {size_t r = 0; for (auto& row : cost_matrix) {
                {size_t c = 0; for (auto& element : row) {
                        if (row_cover.at(r) == 1) {
                            cost_matrix[r][c] += min_value;
                        }
                        if (col_cover.at(c) == 0) {
                            cost_matrix[r][c] -= min_value;
                        }
                        c += 1;
                    }}
                r += 1;
            }}
            step = Step::STEP_FOUR;
        }


        static std::array<Result, N> step_seven(const std::array<std::array<T, N>, N>& cost_matrix,
                                                const std::array<std::array<uint8_t, N>, N>& mask) {
            std::array<Result, N> output;
            {size_t r = 0; for (const auto& row_mask : mask) {
                auto row_col = star_in_row(mask, r);
                output.at(r) = Result(row_col->first, row_col->second, cost_matrix.at(r).at(row_col->second));
                r += 1;
            }}
            return output;
        }


        static std::optional<std::pair<size_t, size_t>> find_uncovered_zero(const std::array<std::array<T, N>, N>& cost_matrix,
                                                                            const std::array<bool, N>& row_cover,
                                                                            const std::array<bool, N>& col_cover) {

            {size_t r = 0; for (auto& row : cost_matrix) {
                {size_t c = 0; for (auto& element : row) {
                        if (element == 0 && row_cover[r] == 0 && col_cover[c] == 0) {
                            return std::make_optional<std::pair<size_t, size_t>>(r, c);
                        }
                        c += 1;
                    }}
                r += 1;
            }}

            return std::nullopt;
        }


        static std::optional<std::pair<size_t, size_t>> star_in_row(const std::array<std::array<uint8_t, N>, N>& mask, size_t row) {
            {size_t c = 0; for (const auto& element : mask.at(row)) {
                if (element == 1) {
                    return std::make_optional<std::pair<size_t, size_t>>(row, c);
                }
                c += 1;
            }}
            return std::nullopt;
        }


        static std::optional<std::pair<size_t, size_t>> star_in_col(const std::array<std::array<uint8_t, N>, N>& mask,
                                                                    size_t col) {

            {size_t r = 0; for (const auto& row : mask) {
                if (row.at(col) == 1) {
                    return std::make_optional<std::pair<size_t, size_t>>(r, col);
                }
                r += 1;
            }}
            return std::nullopt;
        };


        static std::optional<std::pair<size_t, size_t>> prime_in_row(const std::array<std::array<uint8_t, N>, N>& mask,
                                                                    size_t row) {

            {size_t c = 0; for (const auto& element : mask.at(row)) {
                if (mask[row][c] == 2) {
                    return std::make_optional<std::pair<size_t, size_t>>(row, c);
                }
                c += 1;
            }}
            return std::nullopt;
        }


        static void augment_path(std::array<std::array<uint8_t, N>, N>& mask,
                                 std::array<std::array<size_t, 2>, N*2>& path,
                                 size_t path_count) {

            for (size_t p = 0 ; p < path_count ; p++) {
                if (mask[path[p][0]][path[p][1]] == 1) {
                    mask[path[p][0]][path[p][1]] = 0;
                }
                else {
                    mask[path[p][0]][path[p][1]] = 1;
                }
            }
        }


        static void clear_covers(std::array<bool, N>& row_cover,
                                 std::array<bool, N>& col_cover) {
            for (auto& r : row_cover) {r = 0;}
            for (auto& c : col_cover) {c = 0;}
        }


        static void erase_primes(std::array<std::array<uint8_t, N>, N>& mask) {
            for (auto& row_mask : mask) {
                for (auto& element_mask : row_mask) {
                    if (element_mask == 2) {element_mask = 0;}
                }
            }
        }


        static T minimal_value(const std::array<std::array<T, N>, N>& cost_matrix,
                               const std::array<bool, N>& row_cover,
                               const std::array<bool, N>& col_cover) {

            auto min_value = std::numeric_limits<T>::max();
            {size_t r = 0; for (auto& row : cost_matrix) {
                {size_t c = 0; for (auto& element : row) {
                    if (row_cover[r] == 0 && col_cover[c] == 0){
                        if (min_value > cost_matrix[r][c]) {
                            min_value = cost_matrix[r][c];
                        }
                    }
                    c += 1;
                }}
                r += 1;
            }}
            return min_value;
        }
    };
}

#endif //ROBOTICTEMPLATELIBRARY_MUNKRES_H