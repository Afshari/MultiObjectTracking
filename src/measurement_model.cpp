#include "inc/measurement_model.h"

MeasurementModel::MeasurementModel(QObject *parent) : QObject(parent) {

//    this->measurementNoiseCovariance = measurementNoiseCovariance;

}

MatrixXd MeasurementModel::getR() {

    return *this->R;
}

