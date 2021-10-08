
#include <gtest/gtest.h>
#include <QTest>

//#include "tests/inc/testkalmanfilter.h"
//#include "tests/inc/testmeasurementlineargaussian.h"
//#include "tests/inc/testpda.h"
//#include "tests/inc/testmultihypothesis.h"
//#include "tests/inc/testutils.h"

#include "tests/inc/qtest_transition_constant_velocity.h"
#include "tests/inc/qtest_transition_2d_turn.h"
#include "tests/inc/qtest_measurement_constant_velocity.h"
#include "tests/inc/qtest_measurement_range_bearing.h"
#include "tests/inc/qtest_sensor.h"
#include "tests/inc/qtest_hungarian.h"
#include "tests/inc/qtest_utils.h"

int main(int argc, char *argv[]) {

    QTestTransitionConstantVelocity q_transition_constant_velocity;
    QTest::qExec(&q_transition_constant_velocity);

    QTestTransition2dTurn q_transition_2d_turn;
    QTest::qExec(&q_transition_2d_turn);

    QTestMeasurementConstantVelocity q_measurement_constant_velocity;
    QTest::qExec(&q_measurement_constant_velocity);

    QTestMeasurementRangeBearing q_measurement_range_bearing;
    QTest::qExec(&q_measurement_range_bearing);

    QTestSensor q_sensor;
    QTest::qExec(&q_sensor);

    QTestHungarian q_hungarian;
    QTest::qExec(&q_hungarian);

    QTestUtils q_utils;
    QTest::qExec(&q_utils);

//    ::testing::InitGoogleTest(&argc, argv);
//    return RUN_ALL_TESTS();
}









