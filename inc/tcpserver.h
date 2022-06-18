#ifndef _TCP_SERVER_H
#define _TCP_SERVER_H

#include <QObject>
#include <QDebug>
#include <QTcpSocket>
#include <QAbstractSocket>
#include <QNetworkProxy>
#include <QTcpServer>
#include <QThread>
#include <iostream>
#include <Eigen/Dense>

//#include "inc/stategaussian.h"
//#include "inc/measurement_model.h"
//#include "inc/measurementprediction.h"
//#include "inc/measurementlineargaussian.h"
//#include "inc/kalmanfilter.h"
//#include "inc/transition_linear_gaussian.h"
//#include "inc/pda.h"
//#include "inc/singlehypothesis.h"
//#include "inc/multihypothesis.h"
//#include "inc/detection.h"
//#include "inc/pdatracker.h"

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


private:
    QTcpServer server;


signals:

};

#endif // _TCP_SERVER_H
