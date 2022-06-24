#ifndef _DEBUG_SERVER_H_
#define _DEBUG_SERVER_H_

#include <QObject>
#include <QDebug>
#include <QTcpSocket>
#include <QAbstractSocket>
#include <QNetworkProxy>
#include <QTcpServer>
#include <QThread>
#include <iostream>

#include <QQmlApplicationEngine>
#include <QQuickView>
#include <QQmlContext>
#include "ui/inc/ui_connection_handler.h"

#include "inc/input_parser.h"
#include "inc/estimator.h"
#include "inc/measurement_range_bearing.h"
#include "inc/measurement_constant_velocity.h"
#include "inc/transition_constant_velocity.h"
#include "inc/transition_2d_turn.h"
#include "inc/hypothesis.h"
#include "inc/sensor.h"
#include "inc/tracker_nn.h"
#include "inc/tracker.h"
#include "inc/tracker_pda.h"
#include "inc/tracker_gaussian_sum.h"
#include "inc/multi_tracker_gnn.h"
#include "inc/multi_tracker_jpda.h"
#include "inc/multi_tracker_mht.h"

using std::shared_ptr;
using std::make_shared;
using std::tuple;
using std::tie;
using Eigen::Vector2d;
using Eigen::Vector4d;

class DebugServer : public QObject {
    Q_OBJECT
public:
    explicit DebugServer(const QQmlApplicationEngine &engine, QObject *parent = nullptr);

protected:
    shared_ptr<InputParser> inputParser;
    UIConnectionHandler connectionHandler;
    unique_ptr<Tracker> tracker;

    int timerId;
    void timerEvent(QTimerEvent *event);

    void handleCap(const string &data);
    void handlePrune(const string &data);
    void handleNearestNeighbor(const string &data);
    void handleMerge(const string &data);
    void handlePDA(const string &data);
    void handleGaussianSum(const string &data);
    void handleGNN(const string &data);
    void handleJPDA(const string &data);
    void handleMHT(const string &data);
    void handleGroundTruth(const string &data);
    void handleMeasurementsDraw(const string &data);
    void handlePDAwGUI(const string &data);

public slots:
    void start();
    void quit();
    void newConnection();
    void disconnected();
    void readyRead();


private:
    QTcpServer server;

};

#endif // _DEBUG_SERVER_H_
