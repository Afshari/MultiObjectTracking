#ifndef TCPSERVER_H
#define TCPSERVER_H

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
#include "pdatracker.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;


class TCPServer : public QObject
{
    Q_OBJECT
public:
    explicit TCPServer(QObject *parent = nullptr);


public slots:
    void start();
    void quit();
    void newConnection();
    void disconnected();
    void readyRead();

private:

    int dt = 0;

//    VectorXd *recvX;
//    MatrixXd *recvP;
    PDATracker *pdaTracker;
//    QList<Detection *> *recvMeasurements;
//    int sendCounter;

    MeasurementLinearGaussian *measurementModel;
    TransitionLinearGaussian *transitionLinearGaussian;
    KalmanFilter *kalman;
    StateGaussian *prior;
    PDA *pda;




private:
    QTcpServer server;


signals:

};

#endif // TCPSERVER_H
