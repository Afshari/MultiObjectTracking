cmake_minimum_required(VERSION 3.5)

project(MultiObjectTracking LANGUAGES CXX)

find_package(Qt5Test REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Qt5Core)
find_package(Qt5Network)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_VERBOSE_MAKEFILE ON)



include_directories(include)

add_executable(MultiObjectTracking
  main.cpp
  utils.cpp
  detection.cpp
  state.cpp
  stategaussian.cpp
  transitionmodel.cpp
  transitionlineargaussian.cpp
  measurementmodel.cpp
  measurementprediction.cpp
  measurementlineargaussian.cpp
  estimator.cpp
  kalmanfilter.cpp
  associator.cpp
  pda.cpp
  singlehypothesis.cpp
  multihypothesis.cpp

  debugserver.cpp

  Tests/testmeasurementlineargaussian.cpp
  Tests/testkalmanfilter.cpp
  tests/testpda.cpp
  Tests/testmultihypothesis.cpp
)


target_link_libraries(MultiObjectTracking Qt5::Core Qt5::Test Qt5::Network Eigen3::Eigen)


