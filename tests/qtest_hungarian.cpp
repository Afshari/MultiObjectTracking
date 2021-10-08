#include "inc/qtest_hungarian.h"

QTestHungarian::QTestHungarian(QObject *parent) : QObject(parent) {

}


void QTestHungarian::testAlgorithm() {

    double Inf = std::numeric_limits<double>::infinity();
    // please use "-std=c++11" for this initialization of vector.
    vector<vector<double>> costMatrix =
    {   { 28.6949,   266.2001,  Inf,      39.6020,   2.3026,  Inf,     Inf,     Inf },
        { 159.8957,  43.4956,   Inf,      Inf,       Inf,     2.3026,  Inf,     Inf },
        { Inf,       85.7763,   58.6133,  Inf,       Inf,     Inf,     2.3026,  Inf },
        { Inf,       Inf,       Inf,     -10.3871,   Inf,     Inf,     Inf,     2.3026 } };

    Hungarian HungAlgo;
    vector<int> assignment;

    double cost = HungAlgo.Solve(costMatrix, assignment);

    QVERIFY( abs(cost + 3.4794) < 1e-2 );

    for (unsigned int x = 0; x < costMatrix.size(); x++)
        std::cout << x << "," << assignment[x] << "\t";

}
