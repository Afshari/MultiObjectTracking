#include "testkalmanfilter.h"

TestKalmanFilter::TestKalmanFilter(QObject *parent) : QObject(parent) {

}

void TestKalmanFilter::initTestCase() {

    MatrixXd *measurementNoiseCovariance = new MatrixXd(2, 2);
    *measurementNoiseCovariance <<    0.75,  0,
                                      0,     0.75;

    MeasurementLinearGaussian *measure = new MeasurementLinearGaussian(measurementNoiseCovariance);

    TransitionLinearGaussian *transitionLinearGaussian = new TransitionLinearGaussian(0.005);

    kalman = new KalmanFilter(measure, transitionLinearGaussian);
}


void TestKalmanFilter::testxPredictDt0() {

    VectorXd x(4);
    x << -0.16260711, 1, 0.12305334, 1;

    Eigen::Vector4d pVector(1.5, 0.5, 1.5, 0.5);
    MatrixXd p = pVector.asDiagonal();

    StateGaussian prior(&x, &p);

    QVERIFY2(kalman->xPredict(prior, 0).isApprox(x), "x Prediction is Not Correct");
}

void TestKalmanFilter::testxPredictDt1() {

    VectorXd xRef(4);
    xRef << 0.83739289, 1, 1.12305334, 1;

    VectorXd x(4);
    x << -0.16260711, 1, 0.12305334, 1;
    Eigen::Vector4d pVector(1.5, 0.5, 1.5, 0.5);
    MatrixXd p = pVector.asDiagonal();

    StateGaussian prior(&x, &p);

    QVERIFY2(kalman->xPredict(prior, 1).isApprox(xRef), "x Prediction is Not Correct");
}


void TestKalmanFilter::testPPredictDt0() {

    VectorXd x(4);
    x << -0.16260711, 1, 0.12305334, 1;
    Eigen::Vector4d pVector(1.5, 0.5, 1.5, 0.5);
    MatrixXd P = pVector.asDiagonal();

    StateGaussian prior(&x, &P);

    QVERIFY2(kalman->PPredict(prior, 0).isApprox(P), "P Prediction is Not Correct");
}

void TestKalmanFilter::testPPredictDt1() {

    MatrixXd PRef(4, 4);
    PRef << 1.17522955, 0.5025,     0.03433435, 0.,
            0.5025,     0.505,      0.,         0.,
            0.03433435, 0.,         1.17480954, 0.5025,
            0.,         0.,         0.5025,     0.505;

    VectorXd x(4);
    x << -0.16260711, 1, 0.12305334, 1;

    MatrixXd P(4, 4);
    P <<    0.67356288, 0.,         0.03433435, 0.,
            0.,         0.5,        0.,         0.,
            0.03433435, 0.,         0.67314287, 0.,
            0.,         0.,         0.,         0.5;

    StateGaussian prior(&x, &P);

    QVERIFY2(kalman->PPredict(prior, 1).isApprox(PRef, 1e-4), "P Prediction is Not Correct");
}

void TestKalmanFilter::testKalmanGain1() {

    MatrixXd ref(4, 2);
    ref <<   0.66666667, 0.,
             0.,         0.,
             0.,         0.66666667,
             0.,         0.;


    MatrixXd crossCov(4, 2);
    crossCov <<  1.5, 0.,
                 0.,  0.,
                 0.,  1.5,
                 0.,  0.;

    MatrixXd predictCov(2, 2);
    predictCov << 2.25, 0.,
                  0.,   2.25;

    QVERIFY2(kalman->kalmanGain(crossCov, predictCov).isApprox(ref, 1e-4), "Kalman Gain is Not Correct");
}

void TestKalmanFilter::testKalmanGain2() {

    MatrixXd ref(4, 2);
    ref <<   0.6103121,   0.00695117,
             0.26109089, -0.00465728,
             0.00695117,  0.61022706,
            -0.00465728,  0.26114787;

    MatrixXd crossCov(4, 2);
    crossCov <<  1.17522955, 0.03433435,
                 0.5025,     0.,
                 0.03433435, 1.17480954,
                 0.,         0.5025;

    MatrixXd predictCov(2, 2);
    predictCov << 1.92522955, 0.03433435,
                  0.03433435, 1.92480954;


    QVERIFY2(kalman->kalmanGain(crossCov, predictCov).isApprox(ref, 1e-4), "Kalman Gain is Not Correct");
}

void TestKalmanFilter::testPUpdate1() {

    MatrixXd ref(4, 4);
    ref <<  0.5, 0.,  0.,  0.,
            0.,  0.5, 0.,  0.,
            0.,  0.,  0.5, 0.,
            0.,  0.,  0.,  0.5;

    MatrixXd gain(4, 2);
    gain << 0.66666667, 0.,
            0.,         0.,
            0.,         0.66666667,
            0.,         0.;

    MatrixXd pPred(4, 4);
    pPred << 1.5, 0.,  0.,  0.,
             0.,  0.5, 0.,  0.,
             0.,  0.,  1.5, 0.,
             0.,  0.,  0.,  0.5;

    MatrixXd pMeas(2, 2);
    pMeas << 2.25, 0.,
             0.,   2.25;

    QVERIFY2(kalman->PUpdate(gain, pPred, pMeas).isApprox(ref, 1e-4), "Update Covariance is Not Correct");
}


void TestKalmanFilter::testPUpdate2() {

    MatrixXd ref(4, 4);
    ref <<   0.45773407,  0.19581817,  0.00521338, -0.00349296,
             0.19581817,  0.37380183, -0.00349296,  0.00234029,
             0.00521338, -0.00349296,  0.4576703,  0.1958609,
            -0.00349296,  0.00234029,  0.1958609,   0.3737732;

    MatrixXd gain(4, 2);
    gain << 0.6103121,   0.00695117,
            0.26109089, -0.00465728,
            0.00695117,  0.61022706,
           -0.00465728,  0.26114787;

    MatrixXd pPred(4, 4);
    pPred << 1.17522955, 0.5025,     0.03433435, 0.,
             0.5025,     0.505,      0.,         0.,
             0.03433435, 0.,         1.17480954, 0.5025,
             0.,         0.,         0.5025,     0.505;

    MatrixXd pMeas(2, 2);
    pMeas << 1.92522955, 0.03433435,
             0.03433435, 1.92480954;

    QVERIFY2(kalman->PUpdate(gain, pPred, pMeas).isApprox(ref, 1e-4), "Update Covariance is Not Correct");

}

void TestKalmanFilter::testxUpdate1() {

    VectorXd ref(4);
    ref << -3.39994248, 1., -5.81743562, 1.;

    VectorXd xPred(4);
    xPred << 0, 1, 0, 1;

    MatrixXd gain(4, 2);
    gain << 0.66666667, 0.,
            0.,         0.,
            0.,         0.66666667,
            0.,         0.;

    VectorXd xMeas(2);
    xMeas << -5.09991372, -8.72615343;

    VectorXd xMeasPred(2);
    xMeasPred << 0, 0;

    QVERIFY2(kalman->xUpdate(xPred, gain, xMeas, xMeasPred).isApprox(ref, 1e-4), "Update Mean is Not Correct");
}

void TestKalmanFilter::testxUpdate2() {

    VectorXd ref(4);
    ref << 3.91008486, 2.30040957, 2.28423273, 1.45866493;

    VectorXd xPred(4);
    xPred << 0.83739289, 1., 1.12305334, 1.;

    MatrixXd gain(4, 2);
    gain << 0.6103121,   0.00695117,
            0.26109089, -0.00465728,
            0.00695117,  0.61022706,
           -0.00465728,  0.26114787;

    VectorXd xMeas(2);
    xMeas << 5.85099466, 2.96880725;

    VectorXd xMeasPred(2);
    xMeasPred << 0.83739289, 1.12305334;

    QVERIFY2(kalman->xUpdate(xPred, gain, xMeas, xMeasPred).isApprox(ref, 1e-4), "Update Mean is Not Correct");
}

void TestKalmanFilter::testPredictMeasurement1() {

    VectorXd refPredMeas(2);
    refPredMeas << 0.83739289, 1.12305334;

    MatrixXd refCrossCov(4, 2);
    refCrossCov << 1.17522955, 0.03433435,
                   0.5025,     0.,
                   0.03433435, 1.17480954,
                   0.,         0.5025;

    MatrixXd refInnovCov(2, 2);
    refInnovCov << 1.92522955, 0.03433435,
                   0.03433435, 1.92480954;


    VectorXd x(4);
    x << 0.83739289, 1., 1.12305334, 1.;

    MatrixXd P(4, 4);
    P <<    1.17522955, 0.5025    , 0.03433435, 0.,
            0.5025    , 0.505     , 0.        , 0.,
            0.03433435, 0.        , 1.17480954, 0.5025,
            0.        , 0.        , 0.5025    , 0.505;

    StateGaussian state(&x, &P);

    MeasurementPrediction *measurementPrediction;
    measurementPrediction = kalman->predictMeasurement(&state);

    QVERIFY2( (*measurementPrediction->crossCov).isApprox(refCrossCov, 1e-4), "" );
    QVERIFY2( (*measurementPrediction->predMeas).isApprox(refPredMeas, 1e-4), "" );
    QVERIFY2( (*measurementPrediction->innovationCov).isApprox(refInnovCov, 1e-4), "" );
}

void TestKalmanFilter::testUpdate1() {

    VectorXd refx(4);
    refx << -0.3950381, 1., -0.06644286, 1.;
    MatrixXd refP = 0.5 * MatrixXd::Identity(4, 4);


    VectorXd x(4);
    x << 0, 1, 0, 1;
    MatrixXd P(4, 4);
    P << 1.5, 0, 0, 0,
         0, 0.5, 0, 0,
         0, 0, 1.5, 0,
         0, 0, 0, 0.5;
    StateGaussian state(&x, &P);


    MatrixXd mNoiseCovar = 0.75 * MatrixXd::Identity(2, 2);
    MeasurementLinearGaussian measurementModel(&mNoiseCovar);

    VectorXd measurement(2);
    measurement << -0.59255716, -0.0996643;

    VectorXd xPredMeas(2);
    xPredMeas << 0, 0;
    MatrixXd PPredMeas(2, 2);
    PPredMeas << 2.25, 0, 0, 2.25;
    StateGaussian stateMeasurement(&xPredMeas, &PPredMeas);
    MatrixXd crossCov(4, 2);
    crossCov << 1.5, 0., 0. , 0., 0. , 1.5, 0. , 0.;

    MeasurementPrediction measurementPrediction(&stateMeasurement, nullptr, nullptr, &crossCov);


    State* posteriorState = kalman->update(state, measurementModel, measurement, measurementPrediction);

    QVERIFY2( posteriorState->getX().isApprox(refx, 1e-4), "");
    QVERIFY2( posteriorState->getP().isApprox(refP), "");
}










