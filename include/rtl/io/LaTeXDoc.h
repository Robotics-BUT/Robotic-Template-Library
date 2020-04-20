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

#ifndef ROBOTICTEMPLATELIBRARY_IO_LATEXDOC_H
#define ROBOTICTEMPLATELIBRARY_IO_LATEXDOC_H

#include <string>
#include <cstdlib>
#include <fstream>
#include <utility>
#include <functional>

#include "rtl/io/LaTeXTikz2D.h"
#include "rtl/io/LaTeXTikz3D.h"

namespace rtl
{
    //! LaTeX document for high-quality human-readable output.
    /*!
     * LaTeX document allows to aggregate other LaTeX outputs into a single document and provides better unified methods for dealing with compilation commands, temporary files etc. It is possible to automate
     * creation of complex documents such as reports from experiments, or mass data processing with the top visual quality LaTeX can offer.
     */
    class LaTeXDoc
    {
    public:
        //! Constructs LaTeXDoc with given output directory and output name.
        /*!
         * At the time of construction, the output and temporary folders are created if they do not exist and the document file is opened for writing and initialized.
         * @param out_dir output directory path.
         * @param out_name output document name.
         */
        LaTeXDoc(std::string out_dir, std::string out_name) : output_dir(std::move(out_dir)), name(std::move(out_name))
        {
            make_tmp_dir = makeTmpDir;
            make_output_dir = makeOutputDir;
            remove_tmp_dir = removeTmpDir;
            move_final_doc = moveFinalDoc;
            compile_tex = compileTex;

            tmp_dir = output_dir + "_tmp_" + name;
            make_tmp_dir(tmp_dir);
            make_output_dir(output_dir);

            tex_path = tmp_dir + "/" + name + ".tex";
            pdf_path = tmp_dir + "/" + name + ".pdf";
            ofs.open(tex_path, std::ofstream::out | std::ofstream::trunc);
            ofs << "\\documentclass{article}\n";
            ofs << "\\usepackage{graphicx}\n";
            ofs << "\\graphicspath{ {./" + tmp_dir + "/} }\n";
            ofs << "\\begin{document}\n\n";
        }

        //! Finalize the document and destroy *this.
        /*!
         * In the destructor the document is finalized and compiled. After that, the move_final_doc() and remove_tmp_dir() internals are called and *this is finaly destroyed.
         */
        ~LaTeXDoc()
        {
            ofs << "\\end{document}\n";
            ofs.close();
            compile_tex(tex_path, tmp_dir);
            move_final_doc(pdf_path, output_dir);
            remove_tmp_dir(tmp_dir);
        }

        //! Set a callable object taking one std::string argument, which makes a temporary directory with given path.
        /*!
         * During initialization it is called like: \p func(tmp_dir);
         * @tparam T type of the callable object.
         * @param func the callable object.
         */
        template<typename T>
        void setMakeTmpDir(T func) { make_tmp_dir = func; }

        //! Set a callable object taking one std::string argument, which makes an output directory with given path.
        /*!
         * During initialization it is called like: \p func(output_dir);
         * @tparam T type of the callable object.
         * @param func the callable object.
         */
        template<typename T>
        void setMakeOutputDir(T func) { make_output_dir = func; }

        //! Set a callable object taking one std::string argument, which removes the temporary directory with given path.
        /*!
         * During finalization it is called like: \p func(tmp_dir);
         * @tparam T type of the callable object.
         * @param func the callable object.
         */
        template<typename T>
        void setRemoveTmpDir(T func) { remove_tmp_dir = func; }

        //! Set a callable object taking two std::string arguments, which moves the temporary document to the output directory.
        /*!
         * During finalization it is called like: \p func(pdf_path, output_dir);
         * @tparam T type of the callable object.
         * @param func the callable object.
         */
        template<typename T>
        void setMoveFinalDoc(T func) { move_final_doc = func; }

        //! Set a callable object taking two std::string arguments, which makes a temporary directory with given path.
        /*!
         * During compilation it is called like: \p func(tex_path, tmp_dir);
         * @tparam T type of the callable object.
         * @param func the callable object.
         */
        template<typename T>
        void setCompileTex(T func) { compile_tex = func; }

        //! Adds another LaTeX exporter as a figure to the document.
        /*!
         * Currently supports LaTeXTikz2D and LaTeXTikz3D.
         * @tparam LE exporter type.
         * @param le exporter whose output will be added.
         * @param desc description of the figure.
         */
        template<class LE>
        void addLE(LE &le, const std::string &desc = "")
        {
            std::string tmp_file = tmp_dir + "/" + name + "_" + std::to_string(tmp_cnt) + ".tex";
            le.writeTEX(tmp_file);
            compile_tex(tmp_file, tmp_dir);
            ofs << "\\begin{figure}[t]\n";
            ofs << "\\centering\n";
            ofs << "\\includegraphics[width=\\textwidth]{" + tmp_dir + "/" + name + "_" + std::to_string(tmp_cnt) + ".pdf}\n";
            if (!desc.empty())
                ofs << "\\caption{" + desc + "}\n";
            ofs << "\\end{figure}\n\n";
            tmp_cnt++;
        }

        //! Adds a set of another LaTeX exports resulting from repeated function object calls.
        /*!
         * Given a number of columns and a total number of figures to be used forms a grid and calls \p func with \p figs and index of current grid cell. LaTeX export fro \p func call populates the cell.
         * @tparam T callable object type.
         * @param func callable object.
         * @param cols number of columns in the grid.
         * @param figs total number of figures.
         * @param desc description of the whole grid of subfigures.
         */
        template<class T>
        void addGridLE(T &func, size_t cols, size_t figs, const std::string &desc = "")
        {
            ofs << "\\begin{figure}[t]\n";
            ofs << "\\centering\n";
            ofs << "\\begin{tabular}{";
            for (size_t i = 0; i < cols; i++)
                ofs << "c";
            ofs << "}\n";

            for (size_t i = 0; i < figs; i++)
            {
                auto& le = func(figs, i);
                std::string tmp_file = tmp_dir + "/" + name + "_" + std::to_string(tmp_cnt) + ".tex";
                le.writeTEX(tmp_file);
                compile_tex(tmp_file, tmp_dir);
                ofs << "\\includegraphics[width=" + std::to_string(1.0f / float(cols)) + "\\textwidth]{" + tmp_dir + "/" + name + "_" + std::to_string(tmp_cnt) + ".pdf}";
                if (i % cols == cols - 1)
                    ofs << "\\\\\n";
                else
                    ofs << "&\n";
                tmp_cnt++;
            }
            if (!desc.empty())
                ofs << "\\caption{" + desc + "}\n";
            ofs << "\\end{tabular}\n\n";
            ofs << "\\end{figure}\n\n";
        }

    private:
        LaTeXDoc()= default;

        static bool makeTmpDir(const std::string &dir)
        {
            return std::system(("mkdir -p " + dir).c_str()) >= 0;
        }

        static bool makeOutputDir(const std::string &dir)
        {
            return std::system(("mkdir -p " + dir).c_str()) >= 0;
        }

        static bool removeTmpDir(const std::string &dir)
        {
            return std::system(("rm -r " + dir).c_str()) >= 0;
        }

        static bool moveFinalDoc(const std::string &tmp_doc, const std::string &out_dir)
        {
            return std::system(("mv " + tmp_doc + " " + out_dir).c_str()) >= 0;
        }

        static bool compileTex(const std::string &file, const std::string &tmp_dir)
        {
            return std::system(("pdflatex -synctex=1 -interaction=nonstopmode -output-directory=" + tmp_dir + " " + file + " > /dev/null").c_str()) >= 0;
        }

        std::string output_dir, tmp_dir, name, tex_path, pdf_path;
        std::ofstream ofs;
        size_t tmp_cnt{0};
        std::function<bool(const std::string&)> make_tmp_dir, make_output_dir, remove_tmp_dir;
        std::function<bool(const std::string&, const std::string&)> move_final_doc, compile_tex;
    };
}

#endif //ROBOTICTEMPLATELIBRARY_IO_LATEXDOC_H
