#ifndef _DEBUG_RUN_H_
#define _DEBUG_RUN_H_

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
#include "inc/utils.h"
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

class DebugRun : public QObject {
    Q_OBJECT
public:
    explicit DebugRun(const QQmlApplicationEngine &engine, QObject *parent = nullptr);
    void run();

protected:

    UIConnectionHandler connectionHandler;

    int timerId;
    void timerEvent(QTimerEvent *event);

    int nbirths;
    int number_of_steps;
    QString type_of_tracking;
    int data_counter;
    QString data_path;


    shared_ptr<MatrixXd> range_c;
    shared_ptr<Vector2d> s;

    shared_ptr<Sensor> sensor;
    shared_ptr<Transition2dTurn> transition_model;
    shared_ptr<MeasurementRangeBearing> measurement_model;
    shared_ptr<Estimator> estimator;

    shared_ptr<TrackerNN> tracker_nn;
    shared_ptr<TrackerPDA> tracker_pda;
    shared_ptr<TrackerGaussianSum> tracker_gaussian_sum;

    shared_ptr<MultiTrackerGNN> tracker_gnn;
    shared_ptr<MultiTrackerJPDA> tracker_jpda;
    shared_ptr<MultiTrackerMHT> tracker_mht;

    void initSingleTrackers();
    void initMultiTrackers();


    QList<qreal> x_nn,  y_nn;
    QList<qreal> x_pda, y_pda;
    QList<qreal> x_gs,  y_gs;
    QList<qreal> x_ground, y_ground;

    void handleEllipsoidalGating();
    void handlePredictedLikelihood();
    void handleMomentMatching();
    void handlePrune();
    void handleCap();
    void handleNearestNeighbor();
    void handlePDA();
    void handleGaussianSum();
    void handleGNN();
    void handleJPDA();
    void handleMHT();
    void handleIndices();


signals:
    void singleTrackingAddData(QString typeOfDraw, QList<qreal> x, QList<qreal> y);
    void singleTrackingAddItem(QString typeOfItem, qreal x, qreal y);
    void multiTrackingAddItem(QString typeOfItem,  QList<qreal> x, QList<qreal> y);
    void multiTrackingAddData(QString typeOfItem,  QList<qreal> x, QList<qreal> y);
    void setNumberOfBirth(int nbirths);


public slots:
    void receiveFromQml(QString value);
    void qmlCommand(QString type);
};

#endif // _DEBUG_RUN_H_
