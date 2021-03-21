#ifndef TESTPDA_H
#define TESTPDA_H

#include <QObject>
#include <QTest>
#include "pda.h"
#include "stategaussian.h"

class TestPDA : public QObject
{
    Q_OBJECT
public:
    explicit TestPDA(QObject *parent = nullptr);

private slots:
    void testLogPDF();
    void testToPDF();
    void testGetProbability();

signals:

};

#endif // TESTPDA_H
