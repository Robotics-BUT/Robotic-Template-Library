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

#ifndef ROBOTICTEMPLATELIBRARY_LATEXTABLE_H
#define ROBOTICTEMPLATELIBRARY_LATEXTABLE_H

#include <string>
#include <vector>
#include <fstream>

namespace rtl
{
    //! Class for comfortable export of simple tables into LaTeXDoc.
    /*!
     * The API of this class is currently quite spare and for example does not allow merging of cells, multi-page tables etc., however simple layouts work well.
     */
    class LaTeXTable
    {
    public:
        //! Default constructor.
        LaTeXTable() = default;

        //! Default destructor.
        ~LaTeXTable() = default;

        //! Sets column alignment and vertical lines of the table.
        /*!
         * This method sets the tabular environment properties. From:
         * @code
         *      \begin{tabular}{c|c||c||c|c|c}
         *      \end{tabular}
         * @endcode
         * output, parameter \p cs sets the `c|c||c||c|c|c` portion of code.
         * @param cs style string.
         */
        void setColumnStyle(const std::string &cs)
        {
            column_style = cs;
        }

        //! Getter of the column style.
        /*!
         *
         * @return the column style string.
         */
        std::string columnsStyle()
        {
            return column_style;
        }

        //! Sets the first line of the table.
        /*!
         * Since the first line of the table is usually somewhat special, a separate function allows to set it any time during the life of the LaTeXTable object.
         *
         * @param heading_cells a vector of strings with the content of the cells.
         */
        void setHeading(const std::vector<std::string> &heading_cells)
        {
            if (heading_cells.empty())
                return;
            heading = "\t" + heading_cells.front();
            for (size_t i = 1; i != heading_cells.size(); i++)
                heading += " & " + heading_cells[i];
            heading += "\\\\";
        }

        //! Adds a whole new row to the table.
        /*!
         * The row is appended at the end and nothing can be currently inserted before.
         *
         * @param row_cells a vector of strings with the content of the cells.
         */
        void addRow(const std::vector<std::string> &row_cells)
        {
            if (row_cells.empty())
                return;
            std::string new_row = "\n\t" + row_cells.front();
            for (size_t i = 1; i != row_cells.size(); i++)
                new_row += " & " + row_cells[i];
            new_row += "\\\\";
            rows.push_back(new_row);
        }

        //! Adds a horizontal line to the table.
        void addHLine()
        {
            rows.emplace_back("\n\t\\hline");
        }

        //! Writes the table code to the output stream with given description.
        /*!
         *
         * @param ofs output stream for the LaTeX code.
         * @param desc caption of the table.
         */
        void writeTable(std::ofstream &ofs, const std::string &desc) const
        {
            ofs << "\\begin{table}\n";
            ofs << "\\begin{center}\n";
            ofs << "\\begin{tabular}{" << column_style << "}\n";
            ofs << heading;
            for (const auto &r : rows)
                ofs << r;
            ofs << "\n";
            ofs << "\\end{tabular}\n";
            ofs << "\\end{center}\n";
            if (!desc.empty())
                ofs << "\\caption{" + desc + "}\n";
            ofs << "\\end{table}\n\n";
        }

    private:
        std::string heading, column_style;
        std::vector<std::string> rows;

    };
}

#endif //ROBOTICTEMPLATELIBRARY_LATEXTABLE_H
