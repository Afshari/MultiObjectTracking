#include "inc/measurementlineargaussian.h"

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

MatrixXd MeasurementLinearGaussian::S(const MatrixXd &upsilon) {

//    std::cout << "H: \r\n" << H() << std::endl;
//    std::cout << "meas Cross Cov: \r\n" << upsilon << std::endl;
//    std::cout << "Noise Cov: \r\n" << (*measurementNoiseCovariance) << std::endl;
    return H() * upsilon + (*measurementNoiseCovariance);
}

MatrixXd MeasurementLinearGaussian::upsilon(const MatrixXd &predCov) {
    return predCov * H().transpose();
}






