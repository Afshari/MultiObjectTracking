#ifndef TRANSITIONMODEL_H
#define TRANSITIONMODEL_H

#include <QObject>
#include <QDebug>
#include <Eigen/Dense>

using Eigen::MatrixXd;

class TransitionModel : public QObject
{
    Q_OBJECT
public:
    explicit TransitionModel(QObject *parent = nullptr);
    virtual MatrixXd transitionFunction(int dt) { return MatrixXd(); }
    virtual MatrixXd transitionCov(int dt) { return MatrixXd(); }

protected:


private:

signals:

};

#endif // TRANSITIONMODEL_H
