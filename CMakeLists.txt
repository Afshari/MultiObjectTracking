cmake_minimum_required(VERSION 3.5)

project(MultiObjectTracking LANGUAGES CXX)

find_package(Qt5Test REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)


set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5Core)

add_executable(MultiObjectTracking
  main.cpp
  transitionmodel.cpp
  transitionlineargaussian.cpp
)
target_link_libraries(MultiObjectTracking Qt5::Core Qt5::Test Eigen3::Eigen)
