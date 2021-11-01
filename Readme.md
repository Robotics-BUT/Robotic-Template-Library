# Robotic template library (RTL)

![build and test badge](https://github.com/Robotics-BUT/Robotic-Template-Library/actions/workflows/ubuntu-20-04.yml/badge.svg)

A C++ template library for use in robotics. RTL builds on [Standard Template Library](https://en.cppreference.com/w/)  (STL) of the C++ language and the [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)  library for highly optimized linear algebra and related tasks. An original purpose of RTL was to put together an experimentation toolkit for research in robotic mapping and localization, however over the years it became a little more mature and seemed worthwhile to be offered to the community. This way, we want to publish implementations of our algorithms for better reproducibility of our research results, which seems to be [an issue in robotics in general](https://www.nature.com/articles/s42256-019-0066-8). Of course, RTL is not here only for *re*-production, but for production as well. Feel free to built on top of it, adapt it, or just use it as an inspiration for your own implementations of the algorithms discussed. 

## Authors and feedback

For technical feedback, bug reporting, feature proposition etc. please use the GitHub issue system. Opening issues allows all of us to participate on the solution, library related things are kept with its repository and other users may benefit from our communication as well.

For research related communication, paper sharing etc. please use the following contact list:  

- Ales Jelinek (<Ales.Jelinek@ceitec.vutbr.cz>, [ResearchGate](https://www.researchgate.net/profile/Ales_Jelinek)) - main developer and contact person
- Adam Ligocki (<Adam.Ligocki@ceitec.vutbr.cz>, [ResearchGate](https://www.researchgate.net/profile/Adam_Ligocki))
- Ludek Zalud (<Ludek.Zalud@ceitec.vutbr.cz>, [ResearchGate](https://www.researchgate.net/profile/Ludek_Zalud))

## Scope
The content of the library is sorted into several modules according to the functionality they provide.

| **Module** | **Content** |
| ---------------|----------------|
| **Core** | Provides base functionality such as constant definitions independent from STL implementation, type traits, algebraic primitives such as vectors, matrices and quaternions, as well as representations of the most important geometric objects such as line segments, polygon, bounding boxes etc. |
| **Input/Output** | Interface to other formats for storage of presentation of RTL objects. STL compliant command line output is present for logging and the most basic user interaction. LaTeX export can be used to provide high quality 2D and 3D image output and automatic generation of reports from experiments. |
| **Segmentation** | Segmentation module covers algorithms for point cloud processing into continuous clusters of points. This is an important step for outlier removal and guarantee of continuity enabling the fast vectorization algorithms. |
| **Vectorization** | Algorithms for fitting of geometrical primitives to point clouds. Traditional point-eliminating approaches are covered (Reumann-Witkam and Douglas-Peucker algorithms), but the main focus is on total least squares (TLS) fitting. Fast algorithms for lines in 2D and 3D and for planes in 3D are present with many optional features such as approximating polyline construction. or global error optimization. |
| **Transformation** | Geometrical transformations are essential in robotics. This module covers the most important: translation, rotation and rigid transformation of applicable objects from the **Core** module. All transformation work in general in N-dimensional space and can be composed together. Next to transformations themselves, there are also tree and chain structures to manage relations between them and to harness those in a more general template-based code.    
| **Test** | Testing tools for the library, however templates for automated testing of instantiation of other templates and random number generation might come in handy in RTL applications as well. | 

## Usage Example

### Add to Existing Project

The RTL is header only library, so we recomand to add it into the existing project by clonning it as a subsepository in existing project.

```
git submodule add https://github.com/Robotics-BUT/Robotic-Template-Library.git libs/rtl/
git submodule init
git submodule update --recursive
```

and in your CMakeLists.txt file add

```
include_directories(libs/rtl/include/)
```

Now you are able to include RTL headers into you code.

### Run Examples

To make just a brief overview and to make the examples work, clone this repo and run following commands.

```
git clone https://github.com/Robotics-BUT/Robotic-Template-Library.git
cd Robotic-Template-Library/
mkdir build
cd build
cmake .. -DENABLE_EXAMPLES=1
make -j4
```

### Run Tests

To evaluate tests, run:

```
git clone https://github.com/Robotics-BUT/Robotic-Template-Library.git
cd Robotic-Template-Library/
mkdir build
cd build
cmake .. -DENABLE_TESTS=1
make -j4
ctest
```
 

## Documentation

Code reference: https://Robotics-BUT.github.io/Robotic-Template-Library

Online documentation covers RTL library from a programmers point of view. For detailed description of more complex algorithms please refer to the papers in the following section.

## References

If you build on Robotic Template Library in your published research or if you would like to give us credit for our library for some other reason, there is a summary of papers covering notable algorithms presented. Please select an appropriate reference regarding the portion of RTL you have used from the list below.

### Robotic Template Library
This [whitepaper](https://openresearchsoftware.metajnl.com/articles/10.5334/jors.353/) describes the functionality and implementation details of the RTL. We published the paper in the Journal of Open Research Software.

<details>
<summary>BibTeX</summary>
  
```
@article{10.5334/jors.353,
  title={Robotic Template Library},
  author={Jelinek, Ales and Ligocki, Adam and Zalud, Ludek},
  journal={Journal of Open Research Software},
  volume={9},
  number={1},
  year={2021},
  publisher={Ubiquity Press}
}
```
</details>

#### Fast vectorization of an ordered point cloud
Any vectorizer with "FTLS" in its name uses this algorithm, its implementation is in the ExtractorChainFast class and an in-depth description is in the [Fast total least squares vectorization](https://link.springer.com/article/10.1007%2Fs11554-016-0562-6) paper.

<details>
<summary>BibTeX</summary>
  
```
@article{10.1007/s11554-016-0562-6,
author = {Jelinek, Ales and Zalud, Ludek and Jilek, Tomas},
title = {Fast Total Least Squares Vectorization},
year = {2019},
issue_date = {April 2019},
publisher = {Springer-Verlag},
address = {Berlin, Heidelberg},
volume = {16},
number = {2},
issn = {1861-8200},
url = {https://doi.org/10.1007/s11554-016-0562-6},
doi = {10.1007/s11554-016-0562-6},
journal = {J. Real-Time Image Process.},
month = apr,
pages = {459–475},
numpages = {17},
keywords = {Vectorization, Least squares, Point cloud, Linear regression, Robotics}
}
```
</details>

#### Global vectorization error optimization
Any vectorizer with "AFTLS" in its name uses this algorithm, its implementation is in the OptimizerTotalError class and an in-depth description is in the [Augmented postprocessing of the FTLS vectorization algorithm](http://www.scitepress.org/DigitalLibrary/Link.aspx?doi=10.5220/0005962902160223) paper.

<details>
<summary>BibTeX</summary>
  
```
@inproceedings{10.5220/0005962902160223,
author = {Jelinek, Ales and Zalud, Ludek},
title = {Augmented Postprocessing of the FTLS Vectorization Algorithm},
year = {2016},
isbn = {9789897581984},
publisher = {SCITEPRESS - Science and Technology Publications, Lda},
address = {Setubal, PRT},
url = {https://doi.org/10.5220/0005962902160223},
doi = {10.5220/0005962902160223},
booktitle = {Proceedings of the 13th International Conference on Informatics in Control, Automation and Robotics},
pages = {216–223},
numpages = {8},
keywords = {Vectorization, Linear Regression, Point Cloud, Least Squares Fitting, Mobile Robotics.},
location = {Lisbon, Portugal},
series = {ICINCO 2016}
}
```
</details>

## Acknowledgements
RTL library is being developed on [Brno University of Technology](https://www.vutbr.cz/). Initial work was carried out on [Faculty of Electrical Engineering and Communication, Department of Control and Automation](https://www.uamt.feec.vutbr.cz/en) as a part of Ales Jelinek's PhD theses.

## Licence (MIT)
Copyright (c) 2020 Brno University of Technology

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: 

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

