cmake_minimum_required(VERSION 3.10)

project(ICP_KD)

#set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} 
                    include
                    )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ICP_KD
    src/main_KD.cpp
    src/Point.cpp
    src/PLYReader.cpp
    src/KdTree.cpp
    src/ICP_KD.cpp
    src/Process.cpp
)
target_link_libraries(ICP_KD Eigen3::Eigen ${PCL_LIBRARIES})

