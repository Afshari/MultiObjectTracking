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

#include "inc/stategaussian.h"
#include "inc/measurementmodel.h"
#include "inc/measurementprediction.h"
#include "inc/measurementlineargaussian.h"
#include "inc/kalmanfilter.h"
#include "inc/transition_linear_gaussian.h"
#include "inc/pda.h"
#include "inc/singlehypothesis.h"
#include "inc/multihypothesis.h"
#include "inc/detection.h"
#include "inc/pdatracker.h"

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
