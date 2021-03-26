#ifndef MEASUREMENTMODEL_H
#define MEASUREMENTMODEL_H

#include <QObject>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class MeasurementModel : public QObject
{
    Q_OBJECT
public:
    explicit MeasurementModel(QObject *parent = nullptr);
    virtual MatrixXd H() { return MatrixXd(); }
    virtual VectorXd h(const VectorXd &state) { std::ignore = state; return VectorXd(); }

    virtual MatrixXd S(const MatrixXd &upsilon) { std::ignore = upsilon; return MatrixXd(); }
    virtual MatrixXd upsilon(const MatrixXd &predCov) { std::ignore = predCov; return MatrixXd(); }

protected:
    MatrixXd* measurementNoiseCovariance;

signals:

private slots:

};

#endif // MEASUREMENTMODEL_H
