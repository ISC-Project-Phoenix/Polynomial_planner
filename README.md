An opinionated ROS2 C++ node template, optimised for ISC.

# Instructions

1. Clone repo inside your workspaces src directory (Ex. phnx_ws/src)
2. `rosdep install --from-paths . --ignore-src -r -y` to install deps
3. `colcon build` to make sure the repo builds before you mess with it
4. Replace the following in both file names and code exactly and consistently. 
   1. polynomial_planner: Replace with the package name. Use snake case. Ex. `polynomial_planner`
   2. PolynomialPlanner: Replace with the node name. Use Pascal case. Ex. `PolynomialPlanner`
5. `colcon build` again. If it builds, you are done
6. Rename outer folder
7. Review the optional dependencies, and remove what you do not need

# Dependencies
Some common extra dependencies are included. Review them and remove what you don't need.
These are marked with TODO_EXTRA.

# Features

- Unit tests
- ROS-Industrial github CI (will test units and lints)
- C++ formatting via clangformat
- A selection of sane lints
- A single node setup in a multithreaded executor

# File structure

```
.
├── include
│   └── polynomial_planner
│       └── PolynomialPlanner_node.hpp
├── package.xml
├── README.md
├── src
│   ├── PolynomialPlanner.cpp
│   └── PolynomialPlanner_node.cpp
└── tests
    └── unit.cpp
```

PolynomialPlanner_node: Source files for the ROS2 node object itself, and only itself

PolynomialPlanner.cpp: Source for the main function of the node, and only the main function

tests/unit.cpp: Example file for unit tests. This is linked to the node and ros, so both can be used
