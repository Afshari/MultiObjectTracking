#ifndef MEASUREMENTLINEARGAUSSIAN_H
#define MEASUREMENTLINEARGAUSSIAN_H

#include <QObject>
#include <QTest>
#include "measurementmodel.h"
#include <Eigen/Dense>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class MeasurementLinearGaussian : public MeasurementModel
{
public:
    MeasurementLinearGaussian(MatrixXd* measurementNoiseCovariance);
    virtual MatrixXd H() override;
    virtual VectorXd h(const VectorXd &state) override;

    MatrixXd S(const MatrixXd &upsilon) override;
    MatrixXd upsilon(const MatrixXd &predCov) override;



    friend class TestMeasurementLinearGaussian;

private:
     VectorXd state;


};

#endif // MEASUREMENTLINEARGAUSSIAN_H
