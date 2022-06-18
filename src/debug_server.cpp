#include "inc/debug_server.h"

DebugServer::DebugServer(const QQmlApplicationEngine &engine, QObject *parent) : QObject(parent) {

    inputParser = make_shared<InputParser>();

    connect(&server, &QTcpServer::newConnection, this, &DebugServer::newConnection);

    engine.rootContext()->setContextProperty("backend", &connectionHandler);
}


void DebugServer::start() {

    server.listen(QHostAddress::Any, 6060);
    std::cout << "Waiting for a New Connection..." << std::endl;
}

void DebugServer::quit() {

    server.close();
}

void DebugServer::newConnection() {

    QTcpSocket *socket = server.nextPendingConnection();
    connect(socket, &QTcpSocket::disconnected, this, &DebugServer::disconnected);
    connect(socket, &QTcpSocket::readyRead, this, &DebugServer::readyRead);

//    QList<int> x = { 100, 200, 300, 400 };
//    QList<int> y = { 500, 400, 200, 600 };

//     emit connectionHandler.addData(x, y);

    qInfo() << "connected: " << socket;
}

void DebugServer::disconnected() {

    QTcpSocket *socket = qobject_cast<QTcpSocket *>(sender());
    qInfo() << "Disconnected" << socket;
    qInfo() << "Parent" << socket->parent();

    socket->deleteLater();
}


void DebugServer::readyRead() {

    QTcpSocket *socket = qobject_cast<QTcpSocket *>(sender());

    QString received = QString::fromUtf8( socket->readAll() );

    string data = received.toStdString();

    int code = inputParser->getCode(data);
    std::cout << "Code: " << code << std::endl;

    emit connectionHandler.recvData("Hi");

    if(code == 210) { // Handle EllipsoidalGating

        shared_ptr<Vector2d> s = make_shared<Vector2d>( Vector2d( 300, 400 ) );
        shared_ptr<Transition2dTurn> transModel = make_shared<Transition2dTurn>( 1, 1, M_PI/180 );
        shared_ptr<MeasurementConstantVelocity> measModel = make_shared<MeasurementConstantVelocity>( 10 );

        Estimator estimator(measModel, transModel);

        shared_ptr<vector<int>> indices = inputParser->getIndices(data, ":");

        string strState = data.substr( (*indices)[1], (*indices)[2] - (*indices)[1] - 1 );
        string strMeasurements = data.substr( (*indices)[2], data.length() - (*indices)[2] );

        shared_ptr<MatrixXd> measurements = inputParser->getMeasurements(strMeasurements, 0, strMeasurements.length());

        float gating_size = 18.4207;
        shared_ptr<VectorXd> x = inputParser->getOneVectorX(strState, 0, strState.length());
        shared_ptr<MatrixXd> P = make_shared<MatrixXd>( MatrixXd::Identity(4, 4) );
        State state(x, P);

        shared_ptr<ArrayXi> z_gate_idx;
        shared_ptr<MatrixXd> z_gate;
        tie(z_gate_idx, z_gate) = estimator.ellipsoidalGating(state, *measurements, gating_size);

        std::cout << *z_gate_idx << std::endl;
        std::cout << *z_gate << std::endl;

    } else if(code == 211) {    // Handle PredictedLikelihood

        shared_ptr<TransitionConstantVelocity> transModel = make_shared<TransitionConstantVelocity>(1, 2);
        shared_ptr<MeasurementConstantVelocity> measModel = make_shared<MeasurementConstantVelocity>( 5 );

        Estimator estimator(measModel, transModel);

        shared_ptr<vector<int>> indices = inputParser->getIndices(data, ":");

        string strState = data.substr( (*indices)[1], (*indices)[2] - (*indices)[1] - 1 );
        string strMeasurements = data.substr( (*indices)[2], data.length() - (*indices)[2] );

        shared_ptr<VectorXd> x = inputParser->getOneVectorX(strState, 0, strState.length());

        shared_ptr<MatrixXd> measurements = inputParser->getMeasurements(strMeasurements, 0, strMeasurements.length());
        shared_ptr<MatrixXd> P = make_shared<MatrixXd>( MatrixXd::Identity(4, 4) );
        State state(x, P);

        shared_ptr<VectorXd> likelihood = estimator.predictedLikelihood(state, *measurements);
        std::cout << "Likelihood " << *likelihood << std::endl;

    } else if(code == 212) {    // Handle MomentMatching

        shared_ptr<TransitionConstantVelocity> transModel = make_shared<TransitionConstantVelocity>(1, 2);
        shared_ptr<MeasurementConstantVelocity> measModel = make_shared<MeasurementConstantVelocity>( 5 );

        Estimator estimator(measModel, transModel);

        shared_ptr<vector<int>> indices = inputParser->getIndices(data, ":");

        string strX = data.substr( (*indices)[1], (*indices)[2] - (*indices)[1] - 1 );
        string strP = data.substr( (*indices)[2], (*indices)[3] - (*indices)[2] - 1 );
        string strW = data.substr( (*indices)[3], data.length() - (*indices)[3] );

        PtrVecState states = inputParser->getStates(strX, strP);
        shared_ptr<VectorXd> w = inputParser->getOneVectorX(strW, 0, strW.length());

        shared_ptr<State> state = estimator.momentMatching(*w, *states);
        std::cout << "x " << state->getX() << std::endl;
        std::cout << "P " << state->getP() << std::endl;

    } else if(code == 213) {        // Handle prune

        this->handlePrune(data);

    } else if(code == 214) {        // Handle Cap

        this->handleCap(data);

    } else if(code == 215) {        // Handle NN

        this->handleNearestNeighbor(data);

    } else if(code == 216) {        // Handle Merge

        this->handleMerge(data);

    } else if(code == 217) {        // Handle PDA

        this->handlePDA(data);

    } else if(code == 218) {        // Handle Gaussian Sum

        this->handleGaussianSum(data);

    } else if(code == 219) {        // Handle GNN

        this->handleGNN(data);

    } else if(code == 220) {        // handle JPDA

        this->handleJPDA(data);

    } else if(code == 221) {        // handle MHT

        this->handleMHT(data);

    } else if(code == 301) {        // handle GroundTruth

        this->handleGroundTruth(data);

    } else if(code == 302) {        // handle Measurements Draw

        this->handleMeasurementsDraw(data);

    } else if(code == 303) {

        this->handlePDAwGUI(data);
    }

}

void DebugServer::handleCap(const string &data) {

    shared_ptr<vector<int>> indices = inputParser->getIndices(data, ":");

    string strW = data.substr( (*indices)[1], data.length() - (*indices)[1] );
    shared_ptr<VectorXd> w = inputParser->getOneVectorX(strW, 0, strW.length());

    float threshold = 50;
    Hypothesis hypothesis;
    VecState states;

    for(int i = 0; i < w->rows(); i++) {
        shared_ptr<VectorXd> x = make_shared<VectorXd>( VectorXd::Ones(4) * i );
        shared_ptr<MatrixXd> P = make_shared<MatrixXd>( MatrixXd::Ones(4, 4) * i );
        states.push_back( make_shared<State>(x, P) );
    }

    shared_ptr<VectorXd> n_weights;
    PtrVecState n_states;
    tie(n_weights, n_states) = hypothesis.cap(states, *w, threshold);

    std::cout << "weights " << *n_weights << std::endl;

}

void DebugServer::handlePrune(const string &data) {

    shared_ptr<vector<int>> indices = inputParser->getIndices(data, ":");

    string strW = data.substr( (*indices)[1], data.length() - (*indices)[1] );
    shared_ptr<VectorXd> w = inputParser->getOneVectorX(strW, 0, strW.length());

    float threshold = -4.6052;
    Hypothesis hypothesis;
    VecState states;

    for(int i = 0; i < w->rows(); i++) {
        shared_ptr<VectorXd> x = make_shared<VectorXd>( VectorXd::Ones(4) * i );
        shared_ptr<MatrixXd> P = make_shared<MatrixXd>( MatrixXd::Ones(4, 4) * i );
        states.push_back( make_shared<State>(x, P) );
    }

    shared_ptr<VectorXd> n_weights;
    PtrVecState n_states;
    tie(n_weights, n_states) = hypothesis.prune(states, *w, threshold);

    std::cout << *n_weights << std::endl;

}


void DebugServer::handleNearestNeighbor(const string &data) {

    shared_ptr<vector<int>> indices = inputParser->getIndices(data, ":");

    string strX = data.substr( (*indices)[1], (*indices)[2] - (*indices)[1] - 1 );
    string strP = data.substr( (*indices)[2], (*indices)[3] - (*indices)[2] - 1 );
    string strMeasurements = data.substr( (*indices)[3], data.length() - (*indices)[3] );

    MatrixXd measurements = *inputParser->getMeasurements(strMeasurements, 0, strMeasurements.length());

    shared_ptr<State> state = inputParser->getState(strX, strP);
    std::cout << "x " << state->getX() << std::endl;
//    std::cout << "P \r\n" << state.getP() << std::endl;

    MatrixXd range_c(2, 2);
    range_c << -1000,   1000,
               -M_PI,   M_PI;

    shared_ptr<Sensor> sensor = make_shared<Sensor>(0.7, 60, range_c);

    shared_ptr<Transition2dTurn> transModel = make_shared<Transition2dTurn>(1, 1, M_PI/180);

    shared_ptr<Vector2d> s = make_shared<Vector2d>( 300, 400 );
    shared_ptr<MeasurementRangeBearing> measModel = make_shared<MeasurementRangeBearing>(5, M_PI/180, s);

    shared_ptr<Estimator> estimator = make_shared<Estimator>(measModel, transModel);

//    shared_ptr<VectorXd> x = make_shared<VectorXd>( Eigen::Vector<double, 5>(0, 0, 10, 0, M_PI/180) );
//    shared_ptr<MatrixXd> P = make_shared<MatrixXd>( MatrixXd::Identity(5, 5) );
//    (*P)(3, 3) = pow(M_PI/180, 2);
//    (*P)(4, 4) = pow(M_PI/180, 2);
//    shared_ptr<State> state = make_shared<State>(x, P);

    TrackerNN tracker(estimator, state, sensor, 13.8155);
    tracker.step( measurements );


}


void DebugServer::handleMerge(const string &data) {

    shared_ptr<vector<int>> indices = inputParser->getIndices(data, ":");

    string strX = data.substr( (*indices)[1], (*indices)[2] - (*indices)[1] - 1 );
    string strP = data.substr( (*indices)[2], (*indices)[3] - (*indices)[2] - 1 );
    string strW = data.substr( (*indices)[3], data.length() - (*indices)[3] );

    PtrVecState states = inputParser->getStates(strX, strP);
    shared_ptr<VectorXd> w = inputParser->getOneVectorX(strW, 0, strW.length());

    shared_ptr<VectorXd> w_hat;
    PtrVecState states_hat;
    Hypothesis hypothesis;
    tie(w_hat, states_hat) = hypothesis.merge(*states, *w, 1000000);

    for(auto state : *states_hat)
        Utils::printEigen<VectorXd>(state->getX(), "state");

    Utils::printEigen<VectorXd>(*w_hat, "w_hat");


}

void DebugServer::handlePDA(const string &data) {

    shared_ptr<vector<int>> indices = inputParser->getIndices(data, ":");

    string strX = data.substr( (*indices)[1], (*indices)[2] - (*indices)[1] - 1 );
    string strP = data.substr( (*indices)[2], (*indices)[3] - (*indices)[2] - 1 );
    string strMeasurements = data.substr( (*indices)[3], data.length() - (*indices)[3] );

    MatrixXd measurements = *inputParser->getMeasurements(strMeasurements, 0, strMeasurements.length());

    shared_ptr<State> state = inputParser->getState(strX, strP);

    MatrixXd range_c(2, 2);
    range_c << -1000,   1000,
               -M_PI,   M_PI;

    shared_ptr<Sensor> sensor = make_shared<Sensor>(0.7, 60, range_c);

    shared_ptr<Transition2dTurn> transModel = make_shared<Transition2dTurn>(1, 1, M_PI/180);

    shared_ptr<Vector2d> s = make_shared<Vector2d>( 300, 400 );
    shared_ptr<MeasurementRangeBearing> measModel = make_shared<MeasurementRangeBearing>(5, M_PI/180, s);

    shared_ptr<Estimator> estimator = make_shared<Estimator>(measModel, transModel);

    TrackerPDA tracker(estimator, state, sensor, 13.8155, 1e-3);
    tracker.step(measurements);

}


void DebugServer::handleGaussianSum(const string &data) {

    shared_ptr<vector<int>> indices = inputParser->getIndices(data, ":");

    string strX             = data.substr( (*indices)[1], (*indices)[2] - (*indices)[1] - 1 );
    string strP             = data.substr( (*indices)[2], (*indices)[3] - (*indices)[2] - 1 );
    string strMeasurements  = data.substr( (*indices)[3], (*indices)[4] - (*indices)[3] - 1 );
    string strWLogs         = data.substr( (*indices)[4], data.length() - (*indices)[4] );

    MatrixXd measurements = *inputParser->getMeasurements(strMeasurements, 0, strMeasurements.length());

    PtrVecState states = inputParser->getStates(strX, strP);
    shared_ptr<VectorXd> w_logs = inputParser->getOneVectorX(strWLogs, 0, strWLogs.length());

    MatrixXd range_c(2, 2);
    range_c << -1000,   1000,
               -M_PI,   M_PI;

    shared_ptr<Sensor> sensor = make_shared<Sensor>(0.7, 60, range_c);

    shared_ptr<Transition2dTurn> transModel = make_shared<Transition2dTurn>(1, 1, M_PI/180);

    shared_ptr<Vector2d> s = make_shared<Vector2d>( 300, 400 );
    shared_ptr<MeasurementRangeBearing> measModel = make_shared<MeasurementRangeBearing>(5, M_PI/180, s);

    shared_ptr<Estimator> estimator = make_shared<Estimator>(measModel, transModel);


    TrackerGaussianSum tracker(estimator, nullptr, sensor, 13.8155, 50, 1e-3);
    tracker.hypotheses = states;
    tracker.w_logs = w_logs;
    tracker.step(measurements);
}

void DebugServer::handleGNN(const string &data) {

    shared_ptr<vector<int>> indices = inputParser->getIndices(data, ":");

    string strX             = data.substr( (*indices)[1], (*indices)[2] - (*indices)[1] - 1 );
    string strP             = data.substr( (*indices)[2], (*indices)[3] - (*indices)[2] - 1 );
    string strMeasurements  = data.substr( (*indices)[3], data.length() - (*indices)[3] );

    MatrixXd measurements = *inputParser->getMeasurements(strMeasurements, 0, strMeasurements.length());

    PtrVecState states = inputParser->getStates(strX, strP);


    MatrixXd range_c(2, 2);
    range_c << -1000,   1000,
               -M_PI,   M_PI;

    shared_ptr<Sensor> sensor = make_shared<Sensor>(0.9, 10, range_c);

    shared_ptr<Transition2dTurn> transModel = make_shared<Transition2dTurn>(1, 1, M_PI/180);

    shared_ptr<Vector2d> s = make_shared<Vector2d>( 300, 400 );
    shared_ptr<MeasurementRangeBearing> measModel = make_shared<MeasurementRangeBearing>(5, M_PI/180, s);

    shared_ptr<Estimator> estimator = make_shared<Estimator>(measModel, transModel);

    MultiTrackerGNN tracker(estimator, states, sensor, 13.8155, 100, 1e-3);
    tracker.step(measurements);
}


void DebugServer::handleJPDA(const string &data) {

    shared_ptr<vector<int>> indices = inputParser->getIndices(data, ":");

    string strX             = data.substr( (*indices)[1], (*indices)[2] - (*indices)[1] - 1 );
    string strP             = data.substr( (*indices)[2], (*indices)[3] - (*indices)[2] - 1 );
    string strMeasurements  = data.substr( (*indices)[3], data.length() - (*indices)[3] );

    MatrixXd measurements = *inputParser->getMeasurements(strMeasurements, 0, strMeasurements.length());

    PtrVecState states = inputParser->getStates(strX, strP);

    MatrixXd range_c(2, 2);
    range_c << -1000,   1000,
               -M_PI,   M_PI;

    shared_ptr<Sensor> sensor = make_shared<Sensor>(0.9, 10, range_c);

    shared_ptr<Transition2dTurn> transModel = make_shared<Transition2dTurn>(1, 1, M_PI/180);

    shared_ptr<Vector2d> s = make_shared<Vector2d>( 300, 400 );
    shared_ptr<MeasurementRangeBearing> measModel = make_shared<MeasurementRangeBearing>(5, M_PI/180, s);

    shared_ptr<Estimator> estimator = make_shared<Estimator>(measModel, transModel);

    MultiTrackerJPDA tracker(estimator, states, sensor, 13.8155, 100, 1e-3);
    tracker.step(measurements);
}


void DebugServer::handleMHT(const string &data) {

    shared_ptr<vector<int>> indices = inputParser->getIndices(data, ":");

    string strX             = data.substr( (*indices)[1], (*indices)[2] - (*indices)[1] - 1 );
    string strP             = data.substr( (*indices)[2], (*indices)[3] - (*indices)[2] - 1 );
    string strMeasurements  = data.substr( (*indices)[3], (*indices)[4] - (*indices)[3] - 1 );
    string strLogW          = data.substr( (*indices)[4], (*indices)[5] - (*indices)[4] - 1 );
    // string strH             = data.substr( (*indices)[5], (*indices)[6] - data.length() );
    string strH             = data.substr( (*indices)[5], data.length() - (*indices)[5] );

    MatrixXd measurements = *inputParser->getMeasurements(strMeasurements, 0, strMeasurements.length());
    shared_ptr<VectorXd> log_w = inputParser->getOneVectorX(strLogW, 0, strLogW.length());

    NestedPtrVecState states = inputParser->getNestedStates(strX, strP);
    shared_ptr<MatrixXi> H = inputParser->getOneMatrixXi(strH, 0, strH.length());


    MatrixXd range_c(2, 2);
    range_c << -1000,   1000,
               -M_PI,   M_PI;

    shared_ptr<Sensor> sensor = make_shared<Sensor>(0.9, 10, range_c);

    shared_ptr<Transition2dTurn> transModel = make_shared<Transition2dTurn>(1, 1, M_PI/180);

    shared_ptr<Vector2d> s = make_shared<Vector2d>( 300, 400 );
    shared_ptr<MeasurementRangeBearing> measModel = make_shared<MeasurementRangeBearing>(5, M_PI/180, s);

    shared_ptr<Estimator> estimator = make_shared<Estimator>(measModel, transModel);

    MultiTrackerMHT tracker(estimator, nullptr, sensor, 13.8155, 100, 1e-3);
    tracker.H_i     = states;
    tracker.H       = H;
    tracker.log_w   = log_w;
    tracker.step(measurements);
}


void DebugServer::handleGroundTruth(const string &data) {

    shared_ptr<vector<int>> indices = inputParser->getIndices(data, ":");

    string strMeasurements  = data.substr( (*indices)[1], data.length() - (*indices)[1] );
    MatrixXd measurements = *inputParser->getMeasurements(strMeasurements, 0, strMeasurements.length());

    QList<int> x;
    QList<int> y;

    for(int i = 0; i < measurements.cols(); i++) {
        x.push_back(measurements(0, i) / 2);
        y.push_back(measurements(1, i) / 2);
        // std::cout << "index " << i << "  " << measurements(0, i) << ", " << measurements(1, i) << std::endl;
    }
    emit connectionHandler.addData("GroundTruth", x, y);
}

void DebugServer::handleMeasurementsDraw(const string &data) {

    shared_ptr<vector<int>> indices = inputParser->getIndices(data, ":");

    string strMeasurements  = data.substr( (*indices)[1], data.length() - (*indices)[1] );
    MatrixXd measurements = *inputParser->getMeasurements(strMeasurements, 0, strMeasurements.length());

    QList<int> x;
    QList<int> y;

    for(int i = 0; i < measurements.cols(); i++) {
        x.push_back(measurements(0, i) / 2);
        y.push_back(measurements(1, i) / 2);
    }
    emit connectionHandler.addData("Measurements", x, y);
}

void DebugServer::handlePDAwGUI(const string &data) {

    if(tracker == nullptr) {

        VectorXd x(4);
        x << 0, 0, 10, 10;
        MatrixXd P(4, 4);
        P <<    1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        shared_ptr<State> state = make_shared<State>( make_shared<VectorXd>(x), make_shared<MatrixXd>(P) );

        MatrixXd range_c(2, 2);
        range_c << -1000,   1000,
                   -1000,   1000;

        shared_ptr<Sensor> sensor = make_shared<Sensor>(0.9, 10, range_c);

        shared_ptr<TransitionConstantVelocity> transModel = make_shared<TransitionConstantVelocity>(1, 2);
        shared_ptr<MeasurementConstantVelocity> measModel = make_shared<MeasurementConstantVelocity>(10);

        shared_ptr<Estimator> estimator = make_shared<Estimator>(measModel, transModel);

        // tracker = make_unique<TrackerPDA>(estimator, state, sensor, 13.8155, 1e-3);
        tracker= make_unique<TrackerNN>(estimator, state, sensor, 13.8155);
    }

    shared_ptr<vector<int>> indices = inputParser->getIndices(data, ":");
    string strMeasurements = data.substr( (*indices)[1], data.length() - (*indices)[1] );
    MatrixXd measurements = *inputParser->getMeasurements(strMeasurements, 0, strMeasurements.length());
    // std::cout << "measurements " << measurements << std::endl;
    tracker->step(measurements);
    std::cout << "State " << tracker->getX() << std::endl;

    QList<int> x;
    QList<int> y;
    x.push_back( tracker->getX()[0] /2 );
    y.push_back( tracker->getX()[1] /2 );

    emit connectionHandler.addData("Measurements", x, y);
}






