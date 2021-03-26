#include "kalmanfilter.h"

KalmanFilter::KalmanFilter(MeasurementModel *measurementModel, TransitionModel *transitionModel) :
    Estimator(measurementModel, transitionModel) {

//    this->measurementModel = measurementModel;
//    this->transitionModel = transitionModel;

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

    MatrixXd upsilon = this->measurementModel->upsilon(state.getP());
    MatrixXd gain = K(upsilon, measurementPrediction.statePred->getP());
    MatrixXd* P = new MatrixXd( this->PUpdate(gain, state.getP(), measurementPrediction.statePred->getP()) );

    VectorXd* x = new VectorXd( state.getX() + gain * (measurement - measurementPrediction.statePred->getX()) );

    return new StateGaussian(x, P);
}

VectorXd KalmanFilter::xPredict(const State &prior, int dt) {
    return transitionModel->transitionFunction(dt) * prior.getX();
}

MatrixXd KalmanFilter::PPredict(const State &prior, int dt) {

    return transitionModel->transitionFunction(dt) * prior.getP() * transitionModel->transitionFunction(dt).transpose() +
            transitionModel->transitionCov(dt);
}

MatrixXd KalmanFilter::K(const MatrixXd &upsilon, const MatrixXd &predictCov) {

    return upsilon * predictCov.inverse();
}

MatrixXd KalmanFilter::PUpdate(const MatrixXd &gain, const MatrixXd &pPred, const MatrixXd &pMeas) {

    return pPred - gain * pMeas * gain.transpose();
}

MatrixXd KalmanFilter::xUpdate(const VectorXd &xPred, const MatrixXd &gain, const VectorXd &xMeas, const VectorXd &xMeasPred) {

    return xPred + gain * ( xMeas - xMeasPred );
}

MeasurementPrediction* KalmanFilter::predictMeasurement(State *statePred) {

    VectorXd *zPred = new VectorXd(measurementModel->h(statePred->getX()));
//    std::cout << *zPred << std::endl;
//    std::cout << "----------" << std::endl;
//    MatrixXd H = measurementModel->H();
    VectorXd *xzPred = new VectorXd(measurementModel->h(statePred->getX()));
    MatrixXd *upsilon = new MatrixXd(measurementModel->upsilon(statePred->getP()));
//    std::cout << *upsilon << std::endl;
//    std::cout << "----------" << std::endl;
    MatrixXd *S = new MatrixXd(measurementModel->S(*upsilon));
//    std::cout << *S << std::endl;
//    std::cout << "----------" << std::endl;

    return new MeasurementPrediction(statePred, xzPred, zPred, S, upsilon);
}









