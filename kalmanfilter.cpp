#include "kalmanfilter.h"

KalmanFilter::KalmanFilter(MeasurementModel *measurementModel, TransitionModel *transitionModel,
                           QObject *parent) : QObject(parent) {

    this->measurementModel = measurementModel;
    this->transitionModel = transitionModel;
}

void KalmanFilter::predict(StateGaussian *prior, int dt) {

    std::cout << xPredict(prior, dt)  << std::endl;
    std::cout << PPredict(prior, dt)  << std::endl;
//    std::cout << transitionModel->transitionFunction() * stateGaussian->getX()  << std::endl;
}

void KalmanFilter::update() {

}

Eigen::VectorXd KalmanFilter::xPredict(StateGaussian *prior, int dt) {
    return transitionModel->transitionFunction(dt) * prior->getX();
}

Eigen::MatrixXd KalmanFilter::PPredict(StateGaussian *prior, int dt) {

    return transitionModel->transitionFunction(dt) * prior->getP() * transitionModel->transitionFunction(dt).transpose() +
            transitionModel->transitionCov(dt);
}


void KalmanFilter::testxPredictDt0() {

    VectorXd x(4);
    x << -0.16260711, 1, 0.12305334, 1;
    Eigen::Vector4d pVector(1.5, 0.5, 1.5, 0.5);
    MatrixXd p = pVector.asDiagonal();

    StateGaussian prior(&x, &p);

    QVERIFY2(xPredict(&prior, 0).isApprox(x), "x Prediction is Not Correct");
}

void KalmanFilter::testxPredictDt1() {

    VectorXd xRef(4);
    xRef << 0.83739289, 1, 1.12305334, 1;

    VectorXd x(4);
    x << -0.16260711, 1, 0.12305334, 1;
    Eigen::Vector4d pVector(1.5, 0.5, 1.5, 0.5);
    MatrixXd p = pVector.asDiagonal();

    StateGaussian prior(&x, &p);

    QVERIFY2(xPredict(&prior, 1).isApprox(xRef), "x Prediction is Not Correct");
}


void KalmanFilter::testPPredictDt0() {

    VectorXd x(4);
    x << -0.16260711, 1, 0.12305334, 1;
    Eigen::Vector4d pVector(1.5, 0.5, 1.5, 0.5);
    MatrixXd P = pVector.asDiagonal();

    StateGaussian prior(&x, &P);

    QVERIFY2(PPredict(&prior, 0).isApprox(P), "P Prediction is Not Correct");
}

void KalmanFilter::testPPredictDt1() {

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

    QVERIFY2(PPredict(&prior, 1).isApprox(PRef, 1e-4), "P Prediction is Not Correct");
}
