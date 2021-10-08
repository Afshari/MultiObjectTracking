#ifndef SENSOR_H
#define SENSOR_H

#include <QObject>
#include <QtTest/QtTest>
#include <Eigen/Dense>

using Eigen::MatrixXd;

class Sensor : public QObject {
    Q_OBJECT
public:
    explicit Sensor(double P_D, double lambda_c, MatrixXd range_c, QObject *parent = nullptr);

    double get_P_D();
    double get_lambda_c();
    double get_pdf_c();
    double get_intensity();
    MatrixXd get_range_c();


protected:
    double P_D;
    double lambda_c;
    double pdf_c;
    double intensity;
    MatrixXd range_c;

};

#endif // SENSOR_H
