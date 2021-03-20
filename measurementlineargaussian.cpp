#include "measurementlineargaussian.h"

MeasurementLinearGaussian::MeasurementLinearGaussian(MatrixXd* measurementNoiseCovariance) : state(4) {

    this->measurementNoiseCovariance = measurementNoiseCovariance;
    state << 0, 1, 0, 1;

}


MatrixXd MeasurementLinearGaussian::measurementFunction() {
    MatrixXd measurement(2, 4);
    measurement <<  1, 0, 0, 0,
                    0, 0, 1, 0;
    return measurement;
}

MatrixXd MeasurementLinearGaussian::innovationCov(const MatrixXd &measCrossCov) {

    return measurementFunction() * measCrossCov + (*measurementNoiseCovariance);
}

MatrixXd MeasurementLinearGaussian::crossCov(const MatrixXd &predCov) {
    return predCov * measurementFunction().transpose();
}






