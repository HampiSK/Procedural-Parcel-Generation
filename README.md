# Procedural Parcel Generation

An interactive 2D city editor written in C++20 with [raylib](https://www.raylib.com). Design city layouts by drawing roads and generating urban blocks, plots, and houses. Choose from grid-based, organic, radial, or random layouts, with automatic regeneration of affected areas whenever the road network changes.

![Demo](demo.gif)

## How it works

The editor lets you design a city interactively by drawing and editing a connected network of straight or curved roads. The road network defines parcels, which are subdivided into city blocks and individual building lots using configurable procedural rules. Four layout styles grid, organic, radial and random, produce different street patterns and lot arrangements. Lot subdivision balances shape, size, and access to roads, while houses and their surroundings are generated to fit the available space. The system preserves parcel styles and uses deterministic generation to keep results consistent during editing. Changes to the road network or parcel settings regenerate only the affected areas, allowing the city to evolve without rebuilding everything.

Each subdivision style (grid, organic, radial or random) has configurable settings defined in the `City::parcelSettings` array. See `configureParcels()` in `main.cpp` for an example of how to configure these settings.

There are still many improvements (and fixes) that could be made. I may return to this project in the future to bring it to the level of polish and quality it deserves.

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
