#include "kalmanfilter.h"

KalmanFilter::KalmanFilter(MeasurementModel *measurementModel, TransitionModel *transitionModel,
                           QObject *parent) : QObject(parent) {

    this->measurementModel = measurementModel;
    this->transitionModel = transitionModel;
}

void KalmanFilter::predict(const StateGaussian &prior, int dt) {

    std::cout << xPredict(prior, dt)  << std::endl;
    std::cout << PPredict(prior, dt)  << std::endl;
}

void KalmanFilter::update() {

}

VectorXd KalmanFilter::xPredict(const StateGaussian &prior, int dt) {
    return transitionModel->transitionFunction(dt) * prior.getX();
}

MatrixXd KalmanFilter::PPredict(const StateGaussian &prior, int dt) {

    return transitionModel->transitionFunction(dt) * prior.getP() * transitionModel->transitionFunction(dt).transpose() +
            transitionModel->transitionCov(dt);
}

MatrixXd KalmanFilter::kalmanGain(const MatrixXd &crossCov, const MatrixXd &predictCov) {

    return crossCov * predictCov.inverse();
}

MatrixXd KalmanFilter::PUpdate(const MatrixXd &gain, const MatrixXd &pPred, const MatrixXd &pMeas) {

    return pPred - gain * pMeas * gain.transpose();
}

MatrixXd KalmanFilter::xUpdate(const VectorXd &xPred, const MatrixXd &gain, const VectorXd &xMeas, const VectorXd &xMeasPred) {

    return xPred + gain * ( xMeas - xMeasPred );
}

MeasurementPrediction* KalmanFilter::predictMeasurement(StateGaussian *predState) {

    VectorXd *predMeas = new VectorXd(measurementModel->h(predState->getX()));
//    MatrixXd H = measurementModel->H();
    MatrixXd *crossCov = new MatrixXd(measurementModel->crossCov(predState->getP()));
    MatrixXd *innovCov = new MatrixXd(measurementModel->innovationCov(*crossCov));

    return new MeasurementPrediction(predState, predMeas, innovCov, crossCov);
}









