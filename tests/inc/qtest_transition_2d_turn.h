#ifndef QTEST_TRANSITION_2D_TURN_H
#define QTEST_TRANSITION_2D_TURN_H

#include <QObject>
#include <QtTest/QtTest>
#include <iostream>
#include <Eigen/Dense>

#include "inc/transition_2d_turn.h"
#include "inc/utils.h"


class QTestTransition2dTurn : public QObject {
    Q_OBJECT
public:
    explicit QTestTransition2dTurn(QObject *parent = nullptr);

private slots:
    void testQ();
    void testF();
    void testf();


};

#endif // QTEST_TRANSITION_2D_TURN_H
