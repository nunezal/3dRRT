cmake_minimum_required(VERSION 3.12)
project(RRT3D)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find VTK package
find_package(VTK REQUIRED)
message(STATUS "VTK_VERSION: ${VTK_VERSION}")
message(STATUS "VTK_INCLUDE_DIRS: ${VTK_INCLUDE_DIRS}")

include(${VTK_USE_FILE})

# Add RRT3D executable
add_executable(rrt3d RRT3D.cpp)

# Add RRTStar3D executable
add_executable(rrtstar3d RRTStar3D.cpp)

# Link against VTK libraries
target_link_libraries(rrt3d ${VTK_LIBRARIES})
target_link_libraries(rrtstar3d ${VTK_LIBRARIES})

# Installation
install(TARGETS rrt3d rrtstar3d DESTINATION bin)

# Output directory
set_target_properties(rrt3d rrtstar3d PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# Optional: Enable compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(rrt3d PRIVATE -Wall -Wextra)
    target_compile_options(rrtstar3d PRIVATE -Wall -Wextra)
elseif(MSVC)
    target_compile_options(rrt3d PRIVATE /W4)
    target_compile_options(rrtstar3d PRIVATE /W4)
endif()

# Print status message
message(STATUS "Configuration successful - run 'make' to build") 