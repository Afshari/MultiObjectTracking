#include "inc/qtest_transition_constant_velocity.h"

QTestTransitionConstantVelocity::QTestTransitionConstantVelocity(QObject *parent) : QObject(parent) {

}


void QTestTransitionConstantVelocity::testQ() {

    MatrixXd Q_ref(4, 4);
    Q_ref <<    1,     0,     2,     0,
                0,     1,     0,     2,
                2,     0,     4,     0,
                0,     2,     0,     4;

    TransitionConstantVelocity cvTransition(1, 2);

    MatrixXd Q = *cvTransition.Q();

    QVERIFY(Q.isApprox(Q_ref, 1e-4));
}


void QTestTransitionConstantVelocity::testF() {

    MatrixXd F_ref(4, 4);
    F_ref <<     1,     0,     1,     0,
                 0,     1,     0,     1,
                 0,     0,     1,     0,
                 0,     0,     0,     1;
    TransitionConstantVelocity cvTransition(1, 2);

    MatrixXd F = *cvTransition.F( VectorXd() );

    QVERIFY(F.isApprox(F_ref, 1e-4));
}



void QTestTransitionConstantVelocity::testf() {

    Vector4d f_ref(2, 6, 1, 4);

    TransitionConstantVelocity cvTransition(1, 2);

    Vector4d x(1, 2, 1, 4);

    MatrixXd f = *cvTransition.f( VectorXd( x ) );

    QVERIFY(f.isApprox(f_ref, 1e-4));
}











