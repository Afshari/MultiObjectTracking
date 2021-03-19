#include "kalmanfilter.h"

KalmanFilter::KalmanFilter(MeasurementModel *measurementModel, TransitionModel *transitionModel,
                           QObject *parent) : QObject(parent) {

    this->measurementModel = measurementModel;
    this->transitionModel = transitionModel;
}

void KalmanFilter::predict() {

}

void KalmanFilter::update() {

}
