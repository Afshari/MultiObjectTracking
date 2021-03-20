#ifndef MEASUREMENTMODEL_H
#define MEASUREMENTMODEL_H

#include <QObject>
#include <Eigen/Dense>

using Eigen::MatrixXd;

class MeasurementModel : public QObject
{
    Q_OBJECT
public:
    explicit MeasurementModel(QObject *parent = nullptr);
    virtual MatrixXd measurementFunction() { return MatrixXd(); }

protected:
    MatrixXd* measurementNoiseCovariance;

signals:

private slots:
    void test();

};

#endif // MEASUREMENTMODEL_H
