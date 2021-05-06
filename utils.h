#ifndef UTILS_H
#define UTILS_H

#include <QObject>
#include <Eigen/Dense>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;


class Utils : public QObject
{
    Q_OBJECT
public:
    explicit Utils(QObject *parent = nullptr);

    static MatrixXd blkdiag(const MatrixXd& a, int count);
    static VectorXd mean(MatrixXd *means, VectorXd *weights);
    static MatrixXd covar(QList<MatrixXd *> covars, MatrixXd *means, VectorXd *mean, VectorXd *weights);

    static std::string matrixToStr(const Eigen::MatrixXd& mat);
    static std::string vectorToStr(const Eigen::VectorXd& vec);

signals:

};

#endif // UTILS_H
