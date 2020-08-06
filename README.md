# QuickOpp Algorithm Implementation

An implementation of the QuickOpp algorithm described in the paper
"Complete coverage path planning in an agricultural environment"<sup>1</sup>

Completed as a requirement of a Master's Degree in Computer
Science and Robotics at the South Dakota School of Mines and Technology.

Inspired by my work at Raven Industries Inc. in the Applied Technology Division.

### Author
Andrew Stelter

### Major Professor
Dr. Larry Pyeatt <sup>3</sup>

## Background
The problem of Complete Coverage Path Planning (CCPP) is a difficult one to solve.
The goal of a CCPP algorithm is to find a path which can be used to traverse an area such that the entire
area is covered and to find a path such that it is the 'least expensive' by some critera.

There are numerous applications where a best-case solution to the CCPP problem could be used to
perfectly optimize some process - for example, when attempting to determine the path that should be
taken by a Roomba to vacuum a room. CCPP often comes up in the context of precision agriculture, where
finding an efficient path to plant your field reduces the amount of product you expend to plant or spray
and the amount of fuel used to do so.

While there is no known perfect solution to the problem, there are a number of algorithms which
provide sufficiently optimized solutions for most uses.

The algorithm (dubbed QuickOpp) defined by Theresa Driscoll in her 2011 thesis at Iowa State University is one such algorithm which
has an emphasis on agriculture. The QuickOpp algorithm is a redesign of the algorithm designed by Jin<sup>2</sup> with the goal
of reducing computational complexity. This project is an implemetation of Theresa Drisoll's work.

## Definitions
### Ring/Loop
A collection of points ordered such that they define a closed area.

### Polygon
A collection of rings, such that there is one clockwise ring defining an outermost boundary
and zero or more counter-clockwise disjoint rings contained entirely within the outer ring which
define 'holes' or areas that are not to be included.

### Left/Right/Above/Below
These terms are used with respect to the coordinate grid to indicate
* Left - Has an X coordinate which is less than
* Right - Has an X coordinate which is greater than
* Above - Has a Y coordinate which is greater than
* Below - Has a Y coordinate which is greater than

### Swath
A single straight-line path across a shape which is associated with a width, such that
multiple such lines arranged next to each other would cover the entire area.

### Region
A sub-area within a polygon

### DCEL
[Doubly-Connected Edge List](https://en.wikipedia.org/wiki/Doubly_connected_edge_list). Within
this project, each half-edge in a DCEL may be referred to as an 'edge' depending on context, and
each half-edge is not required to have a twin; as the regions considered to be 'outside' the shape
or within holes in the shape are not represented.

## Project Contents
This project is an implementation in three parts
* A C++ library, implemented without dependency on Qt, to perform the QuickOpp algorithm.
* A set of unit tests for the core algorithmic pieces, written using Google Test
* A Qt application which utilizies the QuickOpp library and provides a UI for users to load shapes for processing

### Layout
- Source files are contained under the `src` directory
- Build files are contained under the `qmake` directory
    - ccpp.pro is the root QMake file and can be used to build the entire project
- Test sources are contained under the `gtests` directory
- Any other git repos which are utilized as dependencies are cloned as git submodules under the `submodules` directory

## Building

The project does not have any significantly complicated requirements. There is no install step, so the binaries produced must
be copied or used directly from the build directory.

This project does include git submodules, so be sure to either `git clone --recurse-submodules` or `git submodule update --init --recursive`
after cloning.

### Dependencies
- boost
    - Tested with 1.71.0
    - Mainly Boost.Geometry used, pieces of Boost.Core
- qt
    - Tested with 5.12.0
    - Only required to build the ccpp tool sub-project
    - Uses core, gui, and widgets modules
- geographiclib
    - Included as a git submodule
- googletest
    - Included as a git submodule

### Build Output
Building this project will produce a file structure that looks something like this

```
|- Makefile
|- lib
|  |- libccpp.a
|  |- libgeographiclib.a
|
|- bin
|  |- ccpp-tests
|  |- ccpp-tool
|  |- shape-importers
|     |- [plugins]
|
|- ccpp-tool
|  |- [build files]
|- ccpp-lib
|  |- [build files]
...
```

The `bin` directory contains the executables for the test application and the
demo ccpp application. Currently, there is no install step to prepare the library
portion of the project for inclusion in another project.

#### With Qt Creator
1. Load `qmake/ccpp.pro` in Qt Creator
2. Choose an appropriate shadow build directory (or choose an in-source build)
3. Build the project with the 'build' button

#### From the Command Line
1. Change to the directory you would like to build in
    - This could be the project root, some other directory in the repo, or a wholly separate directory
2. Run `qmake [path/to/qmake/ccpp.pro]`
3. Run `make qmake_all`
4. Run `make`

### Documentation
A Doxygen website for this project can be generated by running `doxygen Doxyfile` in the root of this
repo. This will require that `dot` be in your PATH for generation of association graphs. The documentation
will be generated in `doc/html`.

## Library Design
### Code Layout
As stated above, all source files for the project are placed under the `src` directory. The C++ namespace
structure mirrors the directory structure under `src`, so an object in the file `src/ads/ccpp/decomposer.h`
would be in the namespace `ads::ccpp`. With the exception of `.cpp` files including their associated `.h` file,
all file includes in the project are relative to `src`, so it is necessary only to add `src` to the `INCLUDEPATH`.

For the sake of hiding internal details of classes, some key classes have been elevated a level in the namespace
hierarchy. For example the Dcel class is found in `src/ads/dcel/dcel.h` and has the qualified name `ads::dcel::Dcel`,
but it is also accessible as `ads::Dcel`.

There are three components of `libccpp`: The Dcel (Doubly-Connected Edge-List) component, the Polygon Sweep Line Algorithm
component, and the CCPP algorithm itself. These are separated mainly because Dcels and Sweep-Line algorithms are concepts
existing outside the context of the QuickOpp algorithm and the could potentially be developed into full libraries in their
own right.

Within the ccpp-specific portion of the library, there is a set of interfaces defined in `ads/ccpp/interfaces` and
an implementation of each of the interfaces specific to the QuickOpp algorithm. With the exception of the cost function
interface (`TurnCostCalculatorIf`), the interfaces exist only as a logical separation of duties - that is, the user of
the library does not need to know about their existance, inherit from them, or instantiate concrete implementations of
them.

### Usage
The following code snippet shows how to use the library to execute the QuickOpp algorithm:
```
#include "ads/ccpp/quickopp.h"
#include "ads/ccpp/turn-cost/u-shaped.h"

// Generate/load a polygon of type ads::ccpp::geometry::Polygon2d
// which is a typedef of boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>>

try
{
    const auto ringsAndLines = ads::ccpp::quickOpp(myPolygon, 1, ads::ccpp::turn_cost::UShaped());
} catch (ads::Error& err)
{
    // do something
}
```

This will perform the QuickOpp algorithm to form a series of rings denoting
regions of your polygon that should be traversed separately, and a sequence of lines spaced 1 unit apart for each
ring, denoting the paths that should be used to cover the regions.

For best results, the coordinate system of the points in the polygon should use meters as its units;
however, this is not strictly required. However, until some of the built-in epsilon comparisons are
removed or abstracted, this library does not work well with Very Small polygons (points with a difference of < 0.001)

The call should be wrapped with an exception block because the library is written such that it may throw
an exception in one of two cases
* User input is an invalid shape (it does not pass `boost::is_valid()`)
* One of the assertions during the algorithm fails
    * This is likely an indication that you've discovered an edge case which is handled incorrectly by the library

#### Custom Cost Functions
The last argument to `quickOpp()` is a turn cost function. This parameter should be an instance of some type that
inherits `ads::ccpp::interfaces::TurnCostCalculatorIf`. The calculator will be used to determine the cost value associated
with an edge of the shape if the vehicle is traveling a straight line swathing pattern at some given angle.

The library includes a U-Shaped turn around cost function which implements the cost function defined in the original
QuickOpp paper. It will likely be desirable for users to implement their own cost functions for other types of turns,
like a bow-tie or bulb-shaped turn.

## Algorithmic Description

The QuickOpp algorithm is defined with 2 main steps
1. Decompose the input shape with a modified trapezoidal decomposition algorithm
2. Re-combine adjacent regions when doing so can reduce the overall cost of covering the shape

Obviously, this is a massive simplification of what's going on, but those are effectively the key pieces of the
algorithm. Breaking it down a little further, we can pull out a few more steps related to cost calculations
1. For each direction in a range, find the cost of covering the shape with all swaths going the given direction. Find the cheapest option.
2. Orient the sweep-line for the decomposition step so that it moves across the shape perpendicular to this best overall case (the sweep-line itself is parallel to the best-case direction)
    - This will bias the decomposition to producing long, thin regions which are parallel to the best-case overall direction
    - The sweep-line algorithm should break the shape more often than just at the start and end of inner loops (this is the 'modified' part).
        A region should be created when the adjacent edges of the original shape differ in angle by more than some delta. (10 degrees was chosen for the original paper)
3. Use a depth-first-traversal to visit all regions in the shape, traveling from a region to any adjacent regions (regions that share an edge)
    - For each pair of adjacent regions, consider the cost of combining them vs. keeping them separate
    - If the cost of covering them when combined is cheaper than leaving them separate, then mark them combined

For further details on how the algorithm is defined, see the original paper.

### Modifications

As is the case when creating an implementation of an algorithm, some changes will be made either to suit the needs of the writer or to
resolve ambiguities in the description. In this case, some changes were made for both reasons.

* The original algorithm defines that regions should be created during the sweep-line algorithm both when first
and last encountering an inner loop and when the edges of the shape differ in angle too far. It also indicates that
regions should be created when the sweep line has moved at least as far as a single swath's width, but it is not entirely
clear what this gains. The sweep-line algorithm has been un-modified, and written as a standard trapezoidal-decomposition,
and an extra step has been added to sweep across each region a second time and split it further based on adjacent angles
of edges (checking the distance of the sweep line was removed because it is not clear how that is intended to work). This
change was made to reduce the requirements of the sweep-line algorithm, making it a more standard and objective function which
can more-easily be unit-tested for correctness, and keeping it from getting overly complicated from a bookkeeping perspective.
This should not affect the overall complexity of the algorithm; it just adds a second sweep across the shape, which adds a constant
factor to the runtime. The overall affect on the algorithm is that it may split regions up differently than originally intended.
* The original algorithm states that any region in the decomposed shape will have at most 2 adjacent regions to it, one on
each side. It's unclear exactly how this is accomplished, as it is very easy to see how a region could have more than just
two adjacent regions. It appears that it is accomplished by way of 0-area regions which run along the height of the shape before
splitting off to an upper or lower section (above or below an inner loop). While this may technically work, it is not particularly
practical after the fact; either it becomes necessary to post-process the solution to remove these geometrically invalid areas, or
algorithms following QuickOpp must be able to process them correctly. This library aims to solve this problem by instead relaxing the '2 adjacent regions'
rule and connecting adjacent regions as one might logically expect. While this changes some of the dynamic of the merge step at the end,
it should not affect the runtime significantly.
* The original algorithm is defined as 'building the output DCEL' during the decomposition, and defines points at which
edges should be created, linked, and added to it. All logic related to creating and maintaining the DCEL has been isolated
to a testable set of classes which maintains various DCEL invariants at all times, guaranteeing that the output is valid.
As such, the decomposition step has been slightly modified with regard to how the DCEL is populated.

## References
<sup>1</sup>Driscoll, Theresa Marie, "Complete coverage path planning in an agricultural environment" (2011).Graduate Theses and Dissertations. 12095. https://lib.dr.iastate.edu/etd/12095

<sup>2</sup>Jin, J. Optimal Field Coverage Path Planning on 2D and 3D Surfaces. A dissertation submitted to the graduate faculty in partial fulfillment of the requirements for the degree of Doctor of Philosophy, Agricultural Engineering. Iowa State University, Ames, Iowa, 2009. 

<sup>3</sup>http://www.mcs.sdsmt.edu/lpyeatt/
