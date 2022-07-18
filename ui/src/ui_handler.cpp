#include "ui/inc/ui_handler.h"


// [✓] - Print measurements count
// [✓] - Add timer in qml
// [✓] - Draw Line & Points with timer
// [ ] - Create StateMachine in JS
// [✓] - Find how to handle direction & velocity in Initial State
// [✓] - Define Single Trackers
// [✓] - Write Slot to get Initial Values (x1 & x2)
// [✓] - Write Slot to get Measurements
// [✓] - Write Signal to send back the results
// [✓] - Calculate Execution time of 'Single Tracking'
// [✓] - Show Measurement of each step NOT whole measurements
// [✓] - Clear tracking output after buttonRun Click
// [ ] - Modify StateMachine to handle different situations
// [✓] - Calculate distance between points in JS
// [✓] - Add Temporary Button for calculating distance
// [✓] - Show points for better debugging
// [✓] - Create function for drawing line in QML
// [✓] - Create function for sending measurement to C++
// [✓] - Remove btnDebug & btnCorrect



UIHandler::UIHandler(const QQmlApplicationEngine &engine, QObject *parent) : QObject(parent) {

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    //    timer->start(1000);
    engine.rootContext()->setContextProperty("backend", this);
}


void UIHandler::update() {

    // emit this->recvData(QString::number(counter));
}

void UIHandler::receiveFromQml(QString value) {

    qDebug() << "Received: " << value;
}


void UIHandler::initSingleTrackers(QList<int> lst_x1, QList<int> lst_x2, int lambda_c) {

    int M = 50;
    float P_D = 0.9;
    lambda_c = (lambda_c < 1) ? 1 : lambda_c;

    float T = 1;
    float sigma_omega = 0.017453292519943295;
    float sigma_v = 1;
    float sigma_b = 0.017453292519943295;
    float sigma_r = 5;
    float w_min = 0.001;

    //s = make_shared<Vector2d>(300, 400);
    s = make_shared<Vector2d>(0, 0);
    MatrixXd range_c(2, 2);
    range_c << -2000.0, 2000.0, -M_PI, M_PI;
    this->range_c = make_shared<MatrixXd>(range_c);

    shared_ptr<Sensor> sensor = make_shared<Sensor>(P_D, lambda_c, *this->range_c);
    shared_ptr<Transition2dTurn> transition_model = make_shared<Transition2dTurn>(T, sigma_v, sigma_omega);
    shared_ptr<MeasurementRangeBearing> measurement_model = make_shared<MeasurementRangeBearing>(sigma_r, sigma_b, s);

    VectorXd x1(2);
    x1 << lst_x1[0], lst_x1[1];
    VectorXd x2(2);
    x2 << lst_x2[0], lst_x2[1];
    double velocity = Utils::getVelocity(x1, x2);
    double heading = Utils::getHeading(x1, x2);
    //std::cout << "Velocity: " << velocity << std::endl;
    //std::cout << "Heading: " << heading << std::endl;
    VectorXd x(5);
    x << lst_x1[0], lst_x1[1], velocity, 0, heading;

    MatrixXd P = Eigen::Matrix<double, 5, 1>(1, 1, 1, pow(M_PI/90, 2), 0.5).asDiagonal();
    shared_ptr<State> init_state = make_shared<State>(make_shared<VectorXd>(x), make_shared<MatrixXd>(P));

    shared_ptr<Estimator> estimator = make_shared<Estimator>(measurement_model, transition_model);

    tracker_nn = make_shared<TrackerNN>(estimator, init_state, sensor, GATING_SIZE, w_min);
    tracker_pda = make_shared<TrackerPDA>(estimator, init_state, sensor, GATING_SIZE, w_min);
    tracker_gaussian_sum = make_shared<TrackerGaussianSum>(estimator, init_state, sensor, GATING_SIZE, M, w_min);
}

void UIHandler::getMeasurements( QList<int> xs, QList<int> ys ) {

    MatrixXd z(2, xs.size());
    for(int i = 0; i < xs.size(); i++) {
        z(0, i) = sqrt(pow(xs[i], 2) + pow(ys[i], 2));
        z(1, i) = atan2(double(ys[i]), double(xs[i]));
    }

    tracker_nn->step(z);
    VectorXd state = tracker_nn->getX();
    double x = state(0, 0);
    double y = state(1, 0);
    emit this->singleTrackingAddItem("nearest_neighbor", x, y);

    tracker_pda->step(z);
    state = tracker_pda->getX();
    x = state(0, 0);
    y = state(1, 0);
    emit this->singleTrackingAddItem("pda", x, y);

    tracker_gaussian_sum->step(z);
    state = tracker_gaussian_sum->getX();
    x = state(0, 0);
    y = state(1, 0);
    emit this->singleTrackingAddItem("gaussian_sum", x, y);

    emit this->singleTrackingAddItem("repaint", x, y);
}













