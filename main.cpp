#include <QCoreApplication>
#include "transitionmodel.h"
#include "transitionlineargaussian.h"
#include "measurementlineargaussian.h"

#include <Eigen/Dense>

using Eigen::MatrixXd;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);


    MeasurementLinearGaussian measure;
//    TransitionLinearGaussian transitionLinearGaussian(0.005);

    return a.exec();
}
