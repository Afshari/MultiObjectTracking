#include "inc/qtest_transition_2d_turn.h"

QTestTransition2dTurn::QTestTransition2dTurn(QObject *parent) : QObject(parent) {

}


void QTestTransition2dTurn::testQ() {

    MatrixXd Q_ref(5, 5);
    Q_ref <<  0,         0,         0,         0,         0,
              0,         0,         0,         0,         0,
              0,         0,         1,         0,         0,
              0,         0,         0,         0,         0,
              0,         0,         0,         0,         0.0003;


    Transition2dTurn turnTransition(1, 1, M_PI/180);
    MatrixXd Q = *turnTransition.Q();

    QVERIFY(Q.isApprox(Q_ref, 1e-4));
}

void QTestTransition2dTurn::testF() {

    MatrixXd F_ref(5, 5);
    F_ref << 1,         0,      0.2837,    4.7946,      0,
             0,         1,     -0.9589,    1.4183,      0,
             0,         0,      1,         0,           0,
             0,         0,      0,         1,           1,
             0,         0,      0,         0,           1;

    Eigen::Matrix<double, 5, 1> x(1, 3, 5, 5, 5);
    Transition2dTurn turnTransition(1, 1, M_PI/180);

    MatrixXd F = *turnTransition.F( VectorXd( x ));

    QVERIFY(F.isApprox(F_ref, 1e-4));
}


void QTestTransition2dTurn::testf() {

    Eigen::Matrix<double, 5, 1> f_ref( 2.4183, -1.7946, 5, 10, 5 );

    Eigen::Matrix<double, 5, 1> x(1, 3, 5, 5, 5);
    Transition2dTurn turnTransition(1, 1, M_PI/180);

    VectorXd f = *turnTransition.f( VectorXd( x ));

    QVERIFY(f.isApprox(f_ref, 1e-4));
}








