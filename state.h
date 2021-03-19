#ifndef STATE_H
#define STATE_H

#include <QObject>
#include <Eigen/Dense>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class State : public QObject
{
    Q_OBJECT
public:
    explicit State(QObject *parent = nullptr);
    VectorXd getX();
    MatrixXd getP();

protected:
    VectorXd *x;
    MatrixXd *p;

signals:

};

#endif // STATE_H
