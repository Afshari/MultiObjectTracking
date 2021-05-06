#ifndef PDATRACKER_H
#define PDATRACKER_H

#include <QObject>
#include <QDebug>
#include <QTcpSocket>
#include <QAbstractSocket>
#include <QNetworkProxy>
#include <QTcpServer>
#include <QThread>
#include <iostream>
#include <Eigen/Dense>

#include "stategaussian.h"
#include "measurementmodel.h"
#include "measurementprediction.h"
#include "measurementlineargaussian.h"
#include "kalmanfilter.h"
#include "transitionlineargaussian.h"
#include "pda.h"
#include "singlehypothesis.h"
#include "multihypothesis.h"
#include "detection.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

class PDATracker : public QObject
{
    Q_OBJECT
public:
    explicit PDATracker(QObject *parent = nullptr);

private:

    int dt;
    int scale;

    VectorXd *X;
    MatrixXd *P;
    QList<Detection *> *measurements;

    MeasurementLinearGaussian *measurementModel;

    TransitionLinearGaussian *transitionLinearGaussian;
    KalmanFilter *kalman;
//    StateGaussian *prior;
    PDA *pda;



public:

    void initialize();
    void setScale(int scale);
    void setX(VectorXd *X);
    VectorXd *getX();
    void setP(MatrixXd *P);
    MatrixXd *getP();
    StateGaussian *getCurrentState();
    void setMeasurements(QList<Detection *> * measurements);
    QList<Detection *> *getMeasurements();

    void interpretX(QStringList receivedArr);
    void interpretP(QStringList receivedArr);
    void interpretMeasurements(QStringList receivedArr);

    QString trackLoop();


signals:

};

#endif // PDATRACKER_H
