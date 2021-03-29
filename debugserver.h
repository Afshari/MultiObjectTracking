#ifndef DEBUGSERVER_H
#define DEBUGSERVER_H

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

class DebugServer : public QObject
{
    Q_OBJECT
public:
    explicit DebugServer(QObject *parent = nullptr);

public slots:
    void start();
    void quit();
    void newConnection();
    void disconnected();
    void readyRead();


private:
    int dt = 0;

    VectorXd *recvX;
    MatrixXd *recvP;
    VectorXd *recvXPredMeas;
    MatrixXd *recvS;
    MatrixXd *recvUpsilon;
    VectorXd *recvMeasurement;
    QList<Detection *> *recvMeasurements;
    int sendCounter;

    MatrixXd *recvMeans;
    VectorXd *recvWeights;
    QList<MatrixXd *> *recvCovars;

    MeasurementLinearGaussian *measurementModel;



private:
    QTcpServer server;

signals:

};

#endif // DEBUGSERVER_H
