#ifndef MEASUREMENTLINEARGAUSSIAN_H
#define MEASUREMENTLINEARGAUSSIAN_H

#include <QObject>
#include "measurementmodel.h"
#include <Eigen/Dense>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class MeasurementLinearGaussian : public MeasurementModel
{
public:
    MeasurementLinearGaussian();

private:
     VectorXd state;

};

#endif // MEASUREMENTLINEARGAUSSIAN_H
