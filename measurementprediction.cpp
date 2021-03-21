#include "measurementprediction.h"

MeasurementPrediction::MeasurementPrediction(StateGaussian *predState, VectorXd *predMeas, MatrixXd *innovationCov,
                                             MatrixXd *crossCov,QObject *parent) : QObject(parent) {

    this->predState = predState;
    this->predMeas = predMeas;
    this->innovationCov = innovationCov;
    this->crossCov = crossCov;
}
