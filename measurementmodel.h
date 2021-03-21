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
    virtual VectorXd h(const VectorXd &state) { return VectorXd(); }

    virtual MatrixXd innovationCov(const MatrixXd &measCrossCov) { return MatrixXd(); }
    virtual MatrixXd crossCov(const MatrixXd &predCov) { return MatrixXd(); }

protected:
    MatrixXd* measurementNoiseCovariance;

signals:

private slots:
    void test();

};

#endif // MEASUREMENTMODEL_H
