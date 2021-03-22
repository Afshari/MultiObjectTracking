#ifndef TESTMULTIHYPOTHESIS_H
#define TESTMULTIHYPOTHESIS_H

#include <QObject>
#include <QTest>
#include <QList>

#include "singlehypothesis.h"
#include "multihypothesis.h"

class TestMultiHypothesis : public QObject
{
    Q_OBJECT
public:
    explicit TestMultiHypothesis(QObject *parent = nullptr);

private slots:
    void testNormalize();

signals:

};

#endif // TESTMULTIHYPOTHESIS_H
