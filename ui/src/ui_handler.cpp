#include "ui/inc/ui_handler.h"

UIHandler::UIHandler(QObject *parent) : QObject(parent) {

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    //    timer->start(1000);
}


void UIHandler::update() {

    // emit this->recvData(QString::number(counter));
}

void UIHandler::receiveFromQml(QString value) {

    qDebug() << "Received: " << value;
}


void UIHandler::getMeasurements( QList<int> xs, QList<int> ys ) {

//    for(int i = 0; i < xs.length(); i++) {
//        std::cout << xs[i] << " " << ys[i] << std::endl;
//    }

//    shared_ptr<vector<int>> indices = inputParser->getIndices(data, ":");

//    string strX = data.substr( (*indices)[1], (*indices)[2] - (*indices)[1] - 1 );
//    string strP = data.substr( (*indices)[2], (*indices)[3] - (*indices)[2] - 1 );
//    string strMeasurements = data.substr( (*indices)[3], data.length() - (*indices)[3] );

//    MatrixXd measurements = *inputParser->getMeasurements(strMeasurements, 0, strMeasurements.length());

    VectorXd x(5);
    x << xs[0], ys[0], 1, 1, 0.17;
    MatrixXd P(5, 5);
    P <<    5, 0, 0, 0, 0,
            0, 5, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 0.1;

    shared_ptr<State> state = make_shared<State>( make_shared<VectorXd>(x), make_shared<MatrixXd>(P) );

    MatrixXd range_c(2, 2);
    range_c << -1000,   1000,
               -M_PI,   M_PI;

    shared_ptr<Sensor> sensor = make_shared<Sensor>(0.7, 60, range_c);

    shared_ptr<Transition2dTurn> transModel = make_shared<Transition2dTurn>(1, 1, M_PI/180);
    shared_ptr<Vector2d> s = make_shared<Vector2d>( 300, 400 );
    shared_ptr<MeasurementRangeBearing> measModel = make_shared<MeasurementRangeBearing>(5, M_PI/180, s);
    shared_ptr<Estimator> estimator = make_shared<Estimator>(measModel, transModel);

    TrackerPDA tracker(estimator, state, sensor, 13.8155, 1e-3);

    MatrixXd measurements(2, 1);

    for(int i = 0; i < xs.length(); i++) {
        std::cout << "Measurement " << xs[i] << " " << ys[i] << std::endl;
        measurements << xs[i], ys[i];
        tracker.step(measurements);
    }



}

