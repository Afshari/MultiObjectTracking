#include "kalmanfilter.h"

KalmanFilter::KalmanFilter(MeasurementModel *measurementModel, TransitionModel *transitionModel,
                           QObject *parent) : QObject(parent) {

    this->measurementModel = measurementModel;
    this->transitionModel = transitionModel;
}

State* KalmanFilter::predict(const State &prior, int dt) {

//    std::cout << xPredict(prior, dt)  << std::endl;
//    std::cout << PPredict(prior, dt)  << std::endl;
    VectorXd *x = new VectorXd(xPredict(prior, dt));
    MatrixXd *P = new MatrixXd(PPredict(prior, dt));

    return new StateGaussian(x, P);
}

State* KalmanFilter::update(const State &state, const MeasurementModel &measurementModel,
                          const VectorXd &measurement, const MeasurementPrediction &measurementPrediction) {

    MatrixXd crossCov = this->measurementModel->crossCov(state.getP());
    MatrixXd gain = kalmanGain(crossCov, measurementPrediction.state->getP());
    MatrixXd* P = new MatrixXd( this->PUpdate(gain, state.getP(), measurementPrediction.state->getP()) );

    VectorXd* x = new VectorXd( state.getX() + gain * (measurement - measurementPrediction.state->getX()) );

    return new StateGaussian(x, P);
}

VectorXd KalmanFilter::xPredict(const State &prior, int dt) {
    return transitionModel->transitionFunction(dt) * prior.getX();
}

MatrixXd KalmanFilter::PPredict(const State &prior, int dt) {

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

MeasurementPrediction* KalmanFilter::predictMeasurement(State *predState) {

    VectorXd *predMeas = new VectorXd(measurementModel->h(predState->getX()));
//    std::cout << *predMeas << std::endl;
//    std::cout << "----------" << std::endl;
//    MatrixXd H = measurementModel->H();
    MatrixXd *crossCov = new MatrixXd(measurementModel->crossCov(predState->getP()));
//    std::cout << *crossCov << std::endl;
//    std::cout << "----------" << std::endl;
    MatrixXd *innovCov = new MatrixXd(measurementModel->innovationCov(*crossCov));
//    std::cout << *innovCov << std::endl;
//    std::cout << "----------" << std::endl;

    return new MeasurementPrediction(predState, predMeas, innovCov, crossCov);
}









