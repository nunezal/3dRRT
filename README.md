# 3D Rapidly-Exploring Random Tree (RRT)

A rapidly-exploring random tree (RRT) is a motion planning algorithm designed to efficiently explore and create a path through a 3D space with obstacles. The algorithm works by incrementally building a tree from a start position, extending branches into the space with random sampling and collision checking until it reaches the goal position.

## Features

- 3D path planning with spherical obstacles
- Visualization of the search space and resulting path
- Available in both MATLAB and Python implementations
- Performance optimizations for faster execution
- Configurable goal sampling frequency
- Partial path extension (extends to last valid point before collision)
- Dynamic goal sampling (more exploration early, more exploitation later)

## MATLAB Implementation

### Requirements

- MATLAB (tested on R2019b or newer)

### Usage

1. Open the `RRT3D.m` file in MATLAB
2. Run the script to see the algorithm in action
3. Modify parameters in the script to change:
   - Start and goal positions
   - Number and size of obstacles
   - Search space boundaries
   - Maximum iterations
   - Goal sampling parameters

## Python Implementation

### Requirements

- Python 3.6+
- NumPy
- Matplotlib

### Installation

```bash
pip install numpy matplotlib
```

### Usage

Basic usage:

```bash
python rrt3d.py
```

With command-line options:

```bash
# Disable real-time visualization for faster execution
python rrt3d.py --no-vis

# Change goal sampling frequency (default: 5)
python rrt3d.py --goal-freq 3

# Change number of obstacles (default: 50)
python rrt3d.py --obstacles 30

# Enable dynamic goal sampling
python rrt3d.py --dynamic-sampling

# Combine options
python rrt3d.py --no-vis --obstacles 30 --dynamic-sampling
```

### Command-line Arguments

| Argument             | Description                                                 |
| -------------------- | ----------------------------------------------------------- |
| `--no-vis`           | Disable real-time visualization during search               |
| `--goal-freq N`      | Sample the goal every N iterations (default: 5)             |
| `--obstacles N`      | Set the number of obstacles (default: 50)                   |
| `--dynamic-sampling` | Enable dynamic goal sampling (starts at 10, decreases to 2) |

### Performance Options

The Python implementation includes several options to improve performance:

1. Reduced obstacle count (adjustable with `--obstacles`)
2. Optimized collision checking with fewer interpolation points
3. Option to disable real-time visualization (`--no-vis`)
4. Adjustable goal sampling frequency (`--goal-freq`)
5. Partial path extension to last valid point before collision
6. Dynamic goal sampling for balancing exploration and exploitation

## Visualization Features

- Red star: Start position
- Green star: Goal position
- Blue spheres: Obstacles
- Black lines: Random tree extensions
- Green lines: Goal-directed extensions
- Red line: Final path

## Algorithm Overview

The RRT algorithm works as follows:

1. Initialize a tree with the start position as the root
2. Randomly sample points in the configuration space
3. Find the nearest node in the tree to the sampled point
4. Extend the tree from the nearest node toward the sampled point
   - If a collision is detected, extend only to the last valid point before collision
   - This ensures maximum exploration even in cluttered environments
5. Check for collisions during extension
6. Periodically sample the goal position to bias the search
   - With dynamic sampling, goal sampling frequency increases as the algorithm progresses
   - This balances exploration (early) and exploitation (later) phases
7. Repeat until the tree reaches the goal or maximum iterations

## Customization

You can modify the following parameters:

- `bndry`: The boundaries of the search space [xmin, xmax, ymin, ymax, zmin, zmax]
- `S`: Number of spherical obstacles
- `r`: Radius of obstacles
- `start`: Starting position [x, y, z]
- `goal`: Goal position [x, y, z]
- `branch_length`: Maximum length of each branch in the tree
- `N`: Maximum number of iterations
- `initial_sample_freq`: Initial goal sampling frequency (higher = less frequent)
- `final_sample_freq`: Final goal sampling frequency (lower = more frequent)

## License

This project is open source and available under the MIT License.

## C++ Implementation

### Requirements

To build and run the C++ implementation, you'll need:

- CMake (version 3.12+)
- C++ compiler with C++17 support (GCC, Clang, or MSVC)
- VTK (Visualization Toolkit) library

### Installation of Dependencies

#### Ubuntu/Debian

```bash
sudo apt-get update
sudo apt-get install cmake build-essential
sudo apt-get install libvtk9-dev # or libvtk7-dev depending on your distribution
```

#### macOS (using Homebrew)

```bash
brew install cmake
brew install vtk
```

#### Windows

- Install [CMake](https://cmake.org/download/)
- Install [Visual Studio](https://visualstudio.microsoft.com/downloads/) with C++ support
- Download and install [VTK](https://vtk.org/download/)

### Building and Running

1. Create a build directory:

```bash
mkdir -p 3dRRT/build
cd 3dRRT/build
```

2. Configure and build the project:

```bash
cmake ..
make
```

3. Run the executable:

```bash
./bin/rrt3d
```

### Command Line Options

The C++ implementation supports the same command line options as the Python version:

- `--no-vis`: Disable real-time visualization during search
- `--goal-freq <n>`: Set goal sampling frequency (default: 5)
- `--obstacles <n>`: Set number of obstacles (default: 50)
- `--dynamic-sampling`: Enable dynamic goal sampling

Example:

```bash
./bin/rrt3d --dynamic-sampling --obstacles 30
```

### Interactive Controls

- **Rotate view**: Click and drag with the mouse
- **Zoom**: Right-click and drag up/down or use the scroll wheel
- **Pan**: Middle-click and drag
- **Exit**: Press 'e' key

## Performance Comparison

The C++ implementation offers several advantages over the Python version:

1. **Speed**: Significantly faster execution, especially for complex scenes
2. **Memory usage**: Lower memory footprint for large environments
3. **Visualization performance**: Smoother interactive rendering with VTK

However, the Python implementation may be easier to modify and extend for rapid prototyping.
