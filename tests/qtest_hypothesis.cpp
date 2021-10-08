#include "inc/qtest_hypothesis.h"

QTestHypothesis::QTestHypothesis(QObject *parent) : QObject(parent) {

}


void QTestHypothesis::testNormalizeLogWeights_1() {

    Vector<double, 6> ref_log_w(-2.95924604, -6.81734604, -3.86284604, -0.79034604, -3.32134604, -0.82954604);

    Vector<double, 6> log_w(-3.3337, -7.1918, -4.2373, -1.1648, -3.6958, -1.2040);
    shared_ptr<VectorXd> result_log_w;
    double sum_log_w;
    tie(result_log_w, sum_log_w) = Hypothesis().normalizeLogWeights(log_w);

    QVERIFY(result_log_w->isApprox(ref_log_w, 1e-4));
    QVERIFY( abs(sum_log_w + 0.374453) < 1e-4);
}

void QTestHypothesis::testNormalizeLogWeights_2() {

    Vector<double, 7> ref_log_w(-6.55941914, -1.62621914, -0.55591914, -5.62221914, -5.95551914, -5.34311914, -1.52611914);

    Vector<double, 7> log_w(-6.2373, -1.3041, -0.2338, -5.3001, -5.6334, -5.0210, -1.2040);
    shared_ptr<VectorXd> result_log_w;
    double sum_log_w;
    tie(result_log_w, sum_log_w) = Hypothesis().normalizeLogWeights(log_w);

    QVERIFY(result_log_w->isApprox(ref_log_w, 1e-4));
    QVERIFY( abs(sum_log_w - 0.322119) < 1e-4);
}

void QTestHypothesis::testNormalizeLogWeights_3() {

    Vector<double, 1> ref_log_w(0);

    Vector<double, 1> log_w(-4.3115);
    shared_ptr<VectorXd> result_log_w;
    double sum_log_w;
    tie(result_log_w, sum_log_w) = Hypothesis().normalizeLogWeights(log_w);

    QVERIFY(result_log_w->isApprox(ref_log_w, 1e-4));
    QVERIFY( abs(sum_log_w + 4.3115) < 1e-4);
}


void QTestHypothesis::testNormalizeLogWeights_4() {

    Vector<double, 6> ref_log_w(-2.95924604, -6.81734604, -3.86284604, -0.79034604, -3.32134604, -0.82954604);

    vector<double> log_w = { -3.3337, -7.1918, -4.2373, -1.1648, -3.6958, -1.2040 };
    shared_ptr<VectorXd> result_log_w;
    double sum_log_w;
    tie(result_log_w, sum_log_w) = Hypothesis().normalizeLogWeights(log_w);

    QVERIFY(result_log_w->isApprox(ref_log_w, 1e-4));
    QVERIFY( abs(sum_log_w + 0.374453) < 1e-4);
}












