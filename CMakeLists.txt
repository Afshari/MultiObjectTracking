cmake_minimum_required(VERSION 3.5)

project(MultiObjectTracking LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

MESSAGE( STATUS CMAKE_PREFIX_PATH " = " ${CMAKE_PREFIX_PATH} )

add_definitions(-DRUN_TEST=1)
add_definitions(-DRUN_DEBUG=2)
add_definitions(-DRUN_APP=3)

option(TEST_ENABLE  ""  OFF)
option(DEBUG_ENABLE ""  ON)
option(APP_ENABLE   ""  OFF)

MESSAGE( STATUS "-->> TEST_ENABLE  is " ${TEST_ENABLE}  )
MESSAGE( STATUS "-->> DEBUG_ENABLE is " ${DEBUG_ENABLE} )
MESSAGE( STATUS "-->> APP_ENABLE   is " ${APP_ENABLE}   )

if(APP_ENABLE)
    add_definitions(-DRUN_TYPE=RUN_APP)
    MESSAGE( STATUS "-->> Configure Running App" )
elseif(TEST_ENABLE)
    add_definitions(-DRUN_TYPE=RUN_TEST)
    MESSAGE( STATUS "-->> Configure Running Test" )
elseif(DEBUG_ENABLE)
    add_definitions(-DRUN_TYPE=RUN_DEBUG)
    MESSAGE( STATUS "-->> Configure Running Debug" )
endif()

find_package(Qt5Test REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Qt5Core)
find_package(Qt5Network)

include_directories(inc/stats)

set(SOURCES main.cpp                        ################## SOURCE FILES ##################
    src/input_parser.cpp
    src/state.cpp
    src/estimator.cpp
    src/hypothesis.cpp
    src/sensor.cpp
    src/tracker.cpp
    src/tracker_nn.cpp
    src/tracker_pda.cpp
    src/tracker_gaussian_sum.cpp
    src/multi_tracker.cpp
    src/multi_tracker_gnn.cpp
    src/multi_tracker_jpda.cpp
    src/multi_tracker_mht.cpp
    src/transition_model.cpp
    src/transition_constant_velocity.cpp
    src/transition_2d_turn.cpp
    src/measurement_model.cpp
    src/measurement_constant_velocity.cpp
    src/measurement_range_bearing.cpp
    src/utils.cpp
    libs/hungarian/hungarian.cpp
    libs/murty/murty_state.cpp
    libs/murty/murty.cpp
    inc/input_parser.h                      ################## HEADER FILES ##################
    inc/state.h
    inc/estimator.h
    inc/hypothesis.h
    inc/sensor.h
    inc/tracker.h
    inc/tracker_nn.h
    inc/tracker_pda.h
    inc/tracker_gaussian_sum.h
    inc/multi_tracker.h
    inc/multi_tracker_gnn.h
    inc/multi_tracker_jpda.h
    inc/multi_tracker_mht.h
    inc/transition_model.h
    inc/transition_constant_velocity.h
    inc/transition_2d_turn.h
    inc/measurement_model.h
    inc/measurement_constant_velocity.h
    inc/measurement_range_bearing.h
    inc/utils.h
    libs/hungarian/hungarian.h
    libs/murty/murty_funcs.h
    libs/murty/murty_state.h
    libs/murty/murty.h
)


if(TEST_ENABLE)

    MESSAGE( STATUS "-->> Add Executable for Test" )
    find_package(GTest CONFIG REQUIRED)

    set(SOURCES ${SOURCES}
            tests/qtest_transition_constant_velocity.cpp
            tests/qtest_transition_2d_turn.cpp
            tests/qtest_measurement_constant_velocity.cpp
            tests/qtest_measurement_range_bearing.cpp
            tests/qtest_sensor.cpp
            tests/qtest_hungarian.cpp
            tests/qtest_hypothesis.cpp
            tests/qtest_utils.cpp
            tests/inc/qtest_transition_constant_velocity.h
            tests/inc/qtest_transition_2d_turn.h
            tests/inc/qtest_measurement_constant_velocity.h
            tests/inc/qtest_measurement_range_bearing.h
            tests/inc/qtest_sensor.h
            tests/inc/qtest_hungarian.h
            tests/inc/qtest_hypothesis.h
            tests/inc/qtest_utils.h
        )

    add_executable(TestMultiObjectTracking
        ${SOURCES} )

    target_link_libraries(TestMultiObjectTracking Qt5::Core Qt5::Network Eigen3::Eigen Qt5::Test GTest::gtest GTest::gmock)

elseif(DEBUG_ENABLE)

    MESSAGE( STATUS "-->> Add Executable for Debug" )
    set(SOURCES ${SOURCES}
        src/debug_server.cpp
        inc/debug_server.h
    )

    add_executable(DebugMultiObjectTracking
        ${SOURCES} )

    target_link_libraries(DebugMultiObjectTracking Qt5::Core Qt5::Network Eigen3::Eigen)

else()

    MESSAGE( STATUS "-->> Add Executable for Running App" )

    set(SOURCES ${SOURCES}
        src/tcpserver.cpp
        inc/tcpserver.h
    )

    add_executable(MultiObjectTracking
        ${SOURCES} )
    target_link_libraries(MultiObjectTracking Qt5::Core Qt5::Network Eigen3::Eigen)

endif()

# MESSAGE( STATUS "-->> Source Files for Compilation: " ${SOURCES} )










