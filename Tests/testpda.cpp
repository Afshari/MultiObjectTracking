#include "testpda.h"

TestPDA::TestPDA(QObject *parent) : QObject(parent) {

}

void TestPDA::testLogPDF() {

    double ref = -13.995050;

    VectorXd xDetection(2);
    xDetection <<  6.84504474, -2.05023341;
//    xDetection << 0.02450937, 0.52022081;

    VectorXd xMeasPred(2);
    xMeasPred << 0, 0;

    MatrixXd PMeasPred(2, 2);
    PMeasPred << 2.25, 0.,
                 0.,   2.25;

    PDA pda;

    QVERIFY2( qAbs( pda.logPDF(xDetection, xMeasPred, PMeasPred) - ref ) <= 1e-4, "");
}

void TestPDA::testToPDF() {

    double ref = 0.06659800182665002;
    PDA pda;

    QVERIFY2( qAbs( pda.toPDF(-2.7090807044887333) - ref ) <= 1e-5, "");
}

void TestPDA::testGetProbability() {

    double ref = 0.0003904373547700659;

    PDA pda(0.125, 0.9);

    QVERIFY2( qAbs( pda.getProbability(-9.82232405069974) - ref ) <= 1e-6, "");
}


