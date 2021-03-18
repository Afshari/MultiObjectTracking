#ifndef UTILS_H
#define UTILS_H

#include <QObject>
#include <Eigen/Dense>

using Eigen::MatrixXd;

class Utils : public QObject
{
    Q_OBJECT
public:
    explicit Utils(QObject *parent = nullptr);

    static MatrixXd blkdiag(const MatrixXd& a, int count);

signals:

};

#endif // UTILS_H
