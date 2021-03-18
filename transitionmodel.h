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

private:

signals:

};

#endif // TRANSITIONMODEL_H
