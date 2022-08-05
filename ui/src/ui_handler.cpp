#include "ui/inc/ui_handler.h"


// [✓] - Remove Stop Button
// [✓] - Modify Show Measurements to follow curr_pointer
// [✓] - Complete StateMachine
// [✓] - Remove Unused functions
// [✓] - Remove Timer
// [✓] - Add Page Title
// [ ] - Add README
// [ ] - Add correct names for Tracking formulas
// [ ] - Run Unit Test to see results


UIHandler::UIHandler(const QQmlApplicationEngine &engine, QObject *parent) : QObject(parent) {

    engine.rootContext()->setContextProperty("backend", this);
}

void UIHandler::initSingleTrackers(int x1, int y1, int x2, int y2, int lambda_c) {

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

    VectorXd state_1(2);
    state_1 << x1, y1;
    VectorXd state_2(2);
    state_2 << x2, y2;
    double velocity = Utils::getVelocity(state_1, state_2);
    double heading = Utils::getHeading(state_1, state_2);
    //std::cout << "Velocity: " << velocity << std::endl;
    //std::cout << "Heading: " << heading << std::endl;
    VectorXd x(5);
    x << x1, y1, velocity, 0, heading;

    MatrixXd P = Eigen::Matrix<double, 5, 1>(1, 1, 1, pow(M_PI/90, 2), 0.5).asDiagonal();
    shared_ptr<State> init_state = make_shared<State>(make_shared<VectorXd>(x), make_shared<MatrixXd>(P));

    shared_ptr<Estimator> estimator = make_shared<Estimator>(measurement_model, transition_model);

    tracker_nn = make_shared<TrackerNN>(estimator, init_state, sensor, GATING_SIZE, w_min);
    tracker_pda = make_shared<TrackerPDA>(estimator, init_state, sensor, GATING_SIZE, w_min);
    tracker_gaussian_sum = make_shared<TrackerGaussianSum>(estimator, init_state, sensor, GATING_SIZE, M, w_min);
}

void UIHandler::initMultiTrackers(QList<int> lst_x_1, QList<int> lst_y_1, QList<int> lst_x_2, QList<int> lst_y_2,
                                  int nbirths, int lambda_c) {

    int M = 50;
    float P_D = 0.9;
    lambda_c = (lambda_c < 1) ? 1 : lambda_c;

    this->nbirths = nbirths;
    float T = 1;
    float sigma_omega = 0.017453292519943295;
    float sigma_v = 1;
    float sigma_b = 0.017453292519943295;
    float sigma_r = 5;
    float w_min = 0.001;

    s = make_shared<Vector2d>(0, 0);
    MatrixXd range_c(2, 2);
    range_c << -2000.0, 2000.0, -M_PI, M_PI;
    this->range_c = make_shared<MatrixXd>(range_c);

    shared_ptr<Sensor> sensor = make_shared<Sensor>(P_D, lambda_c, *this->range_c);
    shared_ptr<Transition2dTurn> transition_model = make_shared<Transition2dTurn>(T, sigma_v, sigma_omega);
    shared_ptr<MeasurementRangeBearing> measurement_model = make_shared<MeasurementRangeBearing>(sigma_r, sigma_b, s);

    shared_ptr<Estimator> estimator = make_shared<Estimator>(measurement_model, transition_model);

    PtrVecState states_gnn = make_shared<vector<shared_ptr<State>>>();
    PtrVecState states_jpda = make_shared<vector<shared_ptr<State>>>();
    PtrVecState states_mht = make_shared<vector<shared_ptr<State>>>();
    for(int i = 0; i < nbirths; i++) {

        VectorXd state_1(2);
        state_1 << lst_x_1[i], lst_y_1[i];
        VectorXd state_2(2);
        state_2 << lst_x_2[i], lst_y_2[i];
        double velocity = Utils::getVelocity(state_1, state_2);
        double heading = Utils::getHeading(state_1, state_2);
        VectorXd x(5);
        x << lst_x_1[i], lst_y_1[i], velocity, 0, heading;
        MatrixXd P = Eigen::Matrix<double, 5, 1>(1, 1, 1, pow(M_PI/90, 2), 0.5).asDiagonal();

        states_gnn->push_back(make_shared<State>(make_shared<VectorXd>(x), make_shared<MatrixXd>(P)));
        states_jpda->push_back(make_shared<State>(make_shared<VectorXd>(x), make_shared<MatrixXd>(P)));
        states_mht->push_back(make_shared<State>(make_shared<VectorXd>(x), make_shared<MatrixXd>(P)));
    }

    tracker_gnn = make_shared<MultiTrackerGNN>(estimator, states_gnn, sensor, GATING_SIZE, M, w_min);
    tracker_jpda = make_shared<MultiTrackerJPDA>(estimator, states_jpda, sensor, GATING_SIZE, M, w_min);
    tracker_mht = make_shared<MultiTrackerMHT>(estimator, states_mht, sensor, GATING_SIZE, M, w_min);

    qDebug() << "initMultiTrackers Finished";
    //emit this->multiTrackingAddItem("repaint", QList<qreal>(), QList<qreal>());
}

void UIHandler::singleMeasurements(QList<int> xs, QList<int> ys) {

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

void UIHandler::multiMeasurements(QList<int> xs, QList<int> ys) {

    MatrixXd z(2, xs.size());
    for(int i = 0; i < xs.size(); i++) {
        z(0, i) = sqrt(pow(xs[i], 2) + pow(ys[i], 2));
        z(1, i) = atan2(double(ys[i]), double(xs[i]));
    }

    QList<qreal> x, y;

    tracker_gnn->step(z);
    for(int i = 0; i < this->nbirths; i++) {

        VectorXd state = tracker_gnn->getX(i);
        x.append(state(0, 0));
        y.append(state(1, 0));
    }
    emit this->multiTrackingAddItem("gnn", x, y);

    x.clear();
    y.clear();
    tracker_jpda->step(z);
    for(int i = 0; i < this->nbirths; i++) {
        VectorXd state = tracker_jpda->getX(i);
        x.append(state(0, 0));
        y.append(state(1, 0));
    }
    emit this->multiTrackingAddItem("jpda", x, y);

    x.clear();
    y.clear();
    tracker_mht->step(z);
    for(int i = 0; i < this->nbirths; i++) {
        VectorXd state = tracker_mht->getX(i);
        x.append(state(0, 0));
        y.append(state(1, 0));
    }
    emit this->multiTrackingAddItem("mht", x, y);

    //qDebug() << "multiMeasurements Finished";
    emit this->multiTrackingAddItem("repaint", x, y);
}










