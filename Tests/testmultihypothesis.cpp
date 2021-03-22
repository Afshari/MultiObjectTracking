#include "testmultihypothesis.h"

TestMultiHypothesis::TestMultiHypothesis(QObject *parent) : QObject(parent) {

}

void TestMultiHypothesis::testNormalize() {

    SingleHypothesis single1(nullptr, nullptr, nullptr, nullptr, 0.14500000000000002);
    SingleHypothesis single2(nullptr, nullptr, nullptr, nullptr, 9.760942025866971e-05);
    SingleHypothesis single3(nullptr, nullptr, nullptr, nullptr, 0.47002888186858216);
    SingleHypothesis single4(nullptr, nullptr, nullptr, nullptr, 3.068303843581214e-12);
    SingleHypothesis single5(nullptr, nullptr, nullptr, nullptr, 6.016712927360031e-06);
    SingleHypothesis single6(nullptr, nullptr, nullptr, nullptr, 7.047174264146475e-11);
    SingleHypothesis single7(nullptr, nullptr, nullptr, nullptr, 0.47950561315188);

    QList<SingleHypothesis *> list( { &single1, &single2, &single3, &single4, &single5, &single6, &single7 } );

    QList<double> probs({ 0.13246386836723892, 8.917049238997886e-05, 0.42939202714924385,
                          2.8030303203230023e-12, 5.496531511815753e-06, 6.437903200599261e-11,
                          0.43804943739243324  });


    MultiHypothesis multi(&list);
    multi.normalizeWeights();


    for(int i = 0; i < list.length(); i++) {
        QVERIFY2( qAbs( multi.items->at(i)->probability - probs.at(i) ) < 1e-4, "" );
    }

}
