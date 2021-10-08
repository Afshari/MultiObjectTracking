#ifndef QTEST_HUNGARIAN_H
#define QTEST_HUNGARIAN_H

#include <QObject>
#include <QtTest/QtTest>
#include <iostream>

#include "libs/hungarian/hungarian.h"

using std::vector;

class QTestHungarian : public QObject {
    Q_OBJECT
public:
    explicit QTestHungarian(QObject *parent = nullptr);

private slots:
    void testAlgorithm();

};

#endif // QTEST_HUNGARIAN_H
