# Procedural Parcel Generation

An interactive 2D city editor written in C++20 with [raylib](https://www.raylib.com). Draw straight or curved roads to create parcels that are divided into blocks and road-connected lots, with only affected parcels regenerated when roads, intersections, or parcel types change.

![Demo](demo.gif)

## How it works

1. **Road graph.** Adding a road splits every road it crosses or ends on. When a road is removed, two straight roads that continue each other become one again.
2. **Parcels.** After every change the faces of the graph are traced again, walking from each road to the next one clockwise around the intersection it reaches. Dead ends are dropped and an outline that touches itself is split in two, so every parcel is a simple polygon. A new parcel takes the type and settings of the old parcel at its inside point.
3. **Blocks.** A parcel longer than the largest block gets the street pattern of its type, which cuts it into blocks.
4. **Lots.** A block is split in two along the best of several candidate lines, again and again, until its lots are small enough. The candidates are rated by how balanced the two parts are, how regular their shapes are and how much road they keep. A lot stays only while a road runs along one of its sides and none runs through it.
## Inspired by

- Zebin Chen, Peng Song and F. Peter Ortner. **Hierarchical Co-generation of Parcels and Streets in Urban Modeling.** *Computer Graphics Forum* 43(2), Eurographics 2024. [doi:10.1111/cgf.15053](https://doi.org/10.1111/cgf.15053)
- Carlos A. Vanegas, Tom Kelly, Basil Weber, Jan Halatsch, Daniel G. Aliaga and Pascal Müller. **Procedural Generation of Parcels in Urban Modeling.** *Computer Graphics Forum* 31(2), 681–690, Eurographics 2012. [doi:10.1111/j.1467-8659.2012.03047.x](https://doi.org/10.1111/j.1467-8659.2012.03047.x)
- Farshad Shariatpour, Amir Shakibamanesh and Morteza Rahbar. **Procedural generation as an approach for digital representation of a city.** *Computational Urban Science* 6, 31 (2026). [doi:10.1007/s43762-026-00263-8](https://doi.org/10.1007/s43762-026-00263-8)
- Guoning Chen, Gregory Esch, Peter Wonka, Pascal Müller and Eugene Zhang. **Interactive Procedural Street Modeling.** *ACM Transactions on Graphics* 27(3), Article 103, SIGGRAPH 2008. [Project page](https://www2.cs.uh.edu/~chengu/Publications/streetModeling/street_modeling.html)

## Requirements
- [Visual Studio 2022](https://visualstudio.com) (Not strictly required, but the included setup scripts support this version)
- [Clang](https://clang.llvm.org/) (Linux)
- [Git](https://git-scm.com)

## Getting Started
1. **Setup**
   Navigate to the `/Setup` directory and run the appropriate setup file for your operating system.
   - **Note:** The setup process will execute the Premake executable. If you want to avoid this, download Premake v5.0.0-beta2 (or latest) yourself from the [official release page](https://github.com/premake/premake-core/releases/tag/v5.0.0-beta2).
   - **Note:** The Linux setup has not been thoroughly tested.
2. **Dependencies**
   All dependencies (raylib) will be automatically downloaded and configured (hopefully).
3. **Windows**
   - A VS2022 solution will be created in the root directory.
   - Open the solution, build, and run to execute the project.
4. **Linux**
   - A Makefile will be generated.
   - Build using: `make`
   - Executable is generated here: `Binaries/<platform>/Dev/PPG`

## License
- The repository itself is licensed under the terms described in `LICENSE`.
- Premake is licensed under the BSD 3-Clause license (see `Premake/LICENSE.txt` for details).
- raylib is licensed under the zlib/libpng license (see `Dependencies/Raylib/LICENSE` for details).

**Note:** raylib is included as dependency and licenses are provided in the Dependencies folder. Ensure you comply with their respective license terms when distributing or using this software.
