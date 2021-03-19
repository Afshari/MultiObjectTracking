#include "kalmanfilter.h"

KalmanFilter::KalmanFilter(MeasurementModel *measurementModel, TransitionModel *transitionModel,
                           QObject *parent) : QObject(parent) {

    this->measurementModel = measurementModel;
    this->transitionModel = transitionModel;
}

void KalmanFilter::predict(StateGaussian *prior, int dt) {

//    std::cout << "Mean: \r\n" << stateGaussian->getX() << std::endl;
//    std::cout << "P: " << stateGaussian->getP() << std::endl;

//    std::cout << transitionModel->transitionFunction() << std::endl;

    std::cout << xPredict(prior, dt)  << std::endl;
//    std::cout << transitionModel->transitionFunction() * stateGaussian->getX()  << std::endl;
}

void KalmanFilter::update() {

}

Eigen::VectorXd KalmanFilter::xPredict(StateGaussian *prior, int dt) {
    return transitionModel->transitionFunction(dt) * prior->getX();
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
