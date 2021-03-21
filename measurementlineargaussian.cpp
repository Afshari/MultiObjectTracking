#include "measurementlineargaussian.h"

MeasurementLinearGaussian::MeasurementLinearGaussian(MatrixXd* measurementNoiseCovariance) : state(4) {

    this->measurementNoiseCovariance = measurementNoiseCovariance;
    state << 0, 1, 0, 1;

}


MatrixXd MeasurementLinearGaussian::H() {
    MatrixXd measurement(2, 4);
    measurement <<  1, 0, 0, 0,
                    0, 0, 1, 0;
    return measurement;
}

VectorXd MeasurementLinearGaussian::h(const VectorXd &state) {

    return H() * state;
}

MatrixXd MeasurementLinearGaussian::innovationCov(const MatrixXd &measCrossCov) {

    return H() * measCrossCov + (*measurementNoiseCovariance);
}

MatrixXd MeasurementLinearGaussian::crossCov(const MatrixXd &predCov) {
    return predCov * H().transpose();
}






