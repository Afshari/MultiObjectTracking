#include "measurementprediction.h"

MeasurementPrediction::MeasurementPrediction(State *predState, VectorXd *zPred, MatrixXd *S,
                                             MatrixXd *upsilon,QObject *parent) : QObject(parent) {

    this->state = predState;
    this->zPred = zPred;
    this->S = S;
    this->upsilon = upsilon;
}
