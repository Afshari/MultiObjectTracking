#include "inc/debug_run.h"

// [✓] - Read data from res.txt File
// [✓] - Define 4 arrays in Javascript for 'NearestNeighbor, PDA, GaussianSum, GroundTruth'
// [✓] - Show Measurements in UI
// [✓] - Define function to initialize Trackers
// [✓] - Define initialization variables as Class Members
// [✓] - Write function in MultiTracker to getX by index of Object
// [ ] - Add for-loop to 'initTrackers' to get result of 'tracker_mht' and compare to result in text file
// [✓] - Show result of 'tracker_gnn' in UI


DebugRun::DebugRun(const QQmlApplicationEngine &engine, QObject *parent) : QObject(parent) {

    // engine.rootContext()->setContextProperty("backend", &connectionHandler);
    engine.rootContext()->setContextProperty("backend", this);

    data_path = "4_1";
    // data_path = "sim_c_40_PD_9";
    // data_path = "1_1_c_20_PD_9";
    // data_path = "1_1_c_200_PD_9"; // --> Bug in JPDA
    // data_path = "1_2_c_30_PD_7"; --> Bad Result for GNN & JPDA
    initSingleTrackers();
    // initMultiTrackers();
}

void DebugRun::initSingleTrackers() {

    QMap<QString, QString> init_values;
    Utils::getDataFromFile(QString("debug_data/SOT/%1/init.txt").arg(data_path), init_values);

    qDebug() << init_values;

    int K = init_values["K"].toInt();
    number_of_steps = K;
    int M = init_values["M"].toInt();
    float P_D = init_values["P_D"].toFloat();
    int T = 1;
    int lambda_c = init_values["lambda_c"].toFloat();
    float sigma_omega = init_values["sigmaOmega"].toFloat();
    float sigma_v = init_values["sigmaV"].toFloat();
    float sigma_b = init_values["sigma_b"].toFloat();
    float sigma_r = init_values["sigma_r"].toFloat();
    float w_min = init_values["wmin"].toFloat();

    shared_ptr<VectorXd> init_x = Utils::getVectorXdData(init_values, "initial_state_x", 5);
    shared_ptr<MatrixXd> init_P = Utils::getSquareMatrixXdData(init_values, "initial_state_P", 5);

    s = Utils::getVector2dData(init_values, "s", 2);
    range_c = Utils::getSquareMatrixXdData(init_values, "range_c", 2);


    shared_ptr<Sensor> sensor = make_shared<Sensor>(P_D, lambda_c, *range_c);
    shared_ptr<Transition2dTurn> transition_model = make_shared<Transition2dTurn>(T, sigma_v, sigma_omega);
    shared_ptr<MeasurementRangeBearing> measurement_model = make_shared<MeasurementRangeBearing>(sigma_r, sigma_b, s);

    shared_ptr<State> init_state = make_shared<State>(init_x, init_P);

    shared_ptr<Estimator> estimator = make_shared<Estimator>(measurement_model, transition_model);
    tracker_nn = make_shared<TrackerNN>(estimator, init_state, sensor, GATING_SIZE, w_min);
    tracker_pda = make_shared<TrackerPDA>(estimator, init_state, sensor, GATING_SIZE, w_min);
    tracker_gaussian_sum = make_shared<TrackerGaussianSum>(estimator, init_state, sensor, GATING_SIZE, M, w_min);
}

void DebugRun::initMultiTrackers() {

    QMap<QString, QString> init_values;
    Utils::getDataFromFile(QString("debug_data/MOT/%1/init.txt").arg(data_path), init_values);

    qDebug() << init_values;

    int K = init_values["K"].toInt();
    number_of_steps = K;
    int M = init_values["M"].toInt();
    float P_D = init_values["P_D"].toFloat();
    // float P_G = init_values["P_G"].toFloat();
    int T = init_values["T"].toInt();
    int lambda_c = init_values["lambda_c"].toFloat();
    // int mergeing_threshold = init_values["merging_threshold"].toInt();
    nbirths = init_values["nbirths"].toInt();
    float sigma_omega = init_values["sigmaOmega"].toFloat();
    float sigma_v = init_values["sigmaV"].toFloat();
    float sigma_b = init_values["sigma_b"].toFloat();
    float sigma_r = init_values["sigma_r"].toFloat();
    float w_min = init_values["w_min"].toFloat();

    s = Utils::getVector2dData(init_values, "s", 2);
    range_c = Utils::getSquareMatrixXdData(init_values, "range_c", 2);

    sensor = make_shared<Sensor>(P_D, lambda_c, *range_c);
    transition_model = make_shared<Transition2dTurn>(T, sigma_v, sigma_omega);
    measurement_model = make_shared<MeasurementRangeBearing>(sigma_r, sigma_b, s);
    estimator = make_shared<Estimator>(measurement_model, transition_model);


    PtrVecState states = make_shared<vector<shared_ptr<State>>>();
    for(int i = 1; i <= 4; i++) {

        QMap<QString, QString> state_values;
        Utils::getDataFromFile(QString("debug_data/MOT/%1/init_states_%2.txt").arg(data_path, QString::number(i)), state_values);

        shared_ptr<VectorXd> x = Utils::getVectorXdData(state_values, "var_x", 5);
        shared_ptr<MatrixXd> P = Utils::getSquareMatrixXdData(state_values, "var_P", 5);

        states->push_back(make_shared<State>(x, P));
    }
    tracker_gnn = make_shared<MultiTrackerGNN>(estimator, states, sensor, GATING_SIZE, M, w_min);


    states = make_shared<vector<shared_ptr<State>>>();
    for(int i = 1; i <= 4; i++) {

        QMap<QString, QString> state_values;
        Utils::getDataFromFile(QString("debug_data/MOT/%1/init_states_%2.txt").arg(data_path, QString::number(i)), state_values);

        shared_ptr<VectorXd> x = Utils::getVectorXdData(state_values, "var_x", 5);
        shared_ptr<MatrixXd> P = Utils::getSquareMatrixXdData(state_values, "var_P", 5);

        states->push_back(make_shared<State>(x, P));
    }
    tracker_jpda = make_shared<MultiTrackerJPDA>(estimator, states, sensor, GATING_SIZE, M, w_min);

    states = make_shared<vector<shared_ptr<State>>>();
    for(int i = 1; i <= 4; i++) {

        QMap<QString, QString> state_values;
        Utils::getDataFromFile(QString("debug_data/MOT/%1/init_states_%2.txt").arg(data_path, QString::number(i)), state_values);

        shared_ptr<VectorXd> x = Utils::getVectorXdData(state_values, "var_x", 5);
        shared_ptr<MatrixXd> P = Utils::getSquareMatrixXdData(state_values, "var_P", 5);

        states->push_back(make_shared<State>(x, P));
    }
    tracker_mht = make_shared<MultiTrackerMHT>(estimator, states, sensor, GATING_SIZE, M, w_min);
}

void DebugRun::run() {

    // handleEllipsoidalGating();
    // handlePredictedLikelihood();
    // handleMomentMatching();
    // handlePrune();
    // handleCap();
    // handleNearestNeighbor();
    // handlePDA();
    // handleGaussianSum();
    // handleGNN();
    // handleJPDA();
    // handleMHT();
    // handleIndices();
}


void DebugRun::receiveFromQml(QString value) {

    qDebug() << "Received (Debug Run): " << value;
}

void getPoints(const QMap<QString, QString>& values, const string& field_name, QList<qreal>& x, QList<qreal>& y,
               int x_add_offset = 0, int y_add_offset = 0, float x_mult_offset = 1, float y_mult_offset = 1) {

    if(x.size() > 0)    x.clear();
    if(y.size() > 0)    y.clear();

    MatrixXd mat = Utils::getMeasurementData(values, field_name);

    for(int i = 0; i < mat.cols(); i++) {
        x.append(x_mult_offset * (x_add_offset + mat(0, i)));
        y.append(y_mult_offset * (y_add_offset + mat(1, i)));
    }
}

void polar2Cartesian(const QList<qreal>& r, const QList<qreal>& theta, QList<qreal>& x, QList<qreal>& y,
                     int x_add_offset = 0, int y_add_offset = 0, float x_mult_offset = 1, float y_mult_offset = 1) {

    if(x.size() > 0)    x.clear();
    if(y.size() > 0)    y.clear();

    for(int i = 0; i < r.size(); i++) {
        x.append(x_mult_offset * (x_add_offset + (r[i] * cos(theta[i]))));
        y.append(y_mult_offset * (y_add_offset + (r[i] * sin(theta[i]))));
    }
}

void DebugRun::qmlCommand(QString type) {

    type_of_tracking = type;

    if(type_of_tracking == "single") {

//        QMap<QString, QString> res_values;
//        Utils::getDataFromFile("debug_data/SOT/sim/res.txt", res_values);

//        data_counter = 0;

//        float x_mult_offset = 2;
//        int y_add_offset = 50;
//        getPoints(res_values, "NN_estimated_state", x_nn, y_nn, 0, y_add_offset, x_mult_offset, 1);
//        getPoints(res_values, "GS_estimated_state", x_pda, y_pda, 0, y_add_offset, x_mult_offset, 1);
//        getPoints(res_values, "PDA_estimated_state", x_gs, y_gs, 0, y_add_offset, x_mult_offset, 1);
//        getPoints(res_values, "true_state", x_ground, y_ground, 0, y_add_offset, x_mult_offset, 1);

        emit setNumberOfBirth(1);
        data_counter = 0;
        timerId = startTimer(50);

    } else if(type_of_tracking == "multi") {

        QMap<QString, QString> init_values;
        Utils::getDataFromFile("debug_data/MOT/sim/init.txt", init_values);

        nbirths = init_values["nbirths"].toInt();
        number_of_steps = init_values["K"].toInt();
        s = Utils::getVector2dData(init_values, "s", 2);
        emit setNumberOfBirth(nbirths);
        data_counter = 0;
        timerId = startTimer(50);

    } else if(type_of_tracking == "multi_calc") {

        emit setNumberOfBirth(nbirths);
        data_counter = 0;
        timerId = startTimer(50);

    } else if(type_of_tracking == "stop") {
        killTimer(timerId);
    }
}

void DebugRun::timerEvent(QTimerEvent *event) {

    if(type_of_tracking == "single") {

//        emit this->singleTrackingAddItem("nearest_neighbor", x_nn[data_counter], y_nn[data_counter]);
//        emit this->singleTrackingAddItem("gaussian_sum", x_pda[data_counter], y_pda[data_counter]);
//        emit this->singleTrackingAddItem("pda", x_gs[data_counter], y_gs[data_counter]);
//        emit this->singleTrackingAddItem("true_state", x_ground[data_counter], y_ground[data_counter]);

//        data_counter += 1;
//        if(data_counter >= x_nn.size())
//           killTimer(timerId);

        data_counter += 1;
        QList<qreal> x, y, r, theta;

        float x_mult_offset = 0.5;
        float y_mult_offset = 0.5;
        int x_add_offset = 1700;
        int y_add_offset = 1200;

        qDebug() << "i: " << data_counter << "--------------------------";
        QMap<QString, QString> data_values;
        Utils::getDataFromFile(QString("debug_data/SOT/%1/%2.txt").arg(data_path, QString::number(data_counter)), data_values);

        MatrixXd z = Utils::getMeasurementData(data_values);

        tracker_nn->step(z);
        VectorXd state = tracker_nn->getX();
        x.append(x_mult_offset * (x_add_offset + state(0, 0)));
        y.append(y_mult_offset * (y_add_offset + state(1, 0)));
        emit this->multiTrackingAddItem("gnn", x, y);

        x.clear();
        y.clear();
        tracker_pda->step(z);
        state = tracker_pda->getX();
        x.append(x_mult_offset * (x_add_offset + state(0, 0)));
        y.append(y_mult_offset * (y_add_offset + state(1, 0)));
        emit this->multiTrackingAddItem("jpda", x, y);

        x.clear();
        y.clear();
        tracker_gaussian_sum->step(z);
        state = tracker_gaussian_sum->getX();
        x.append(x_mult_offset * (x_add_offset + state(0, 0)));
        y.append(y_mult_offset * (y_add_offset + state(1, 0)));
        emit this->multiTrackingAddItem("mht", x, y);

        getPoints(data_values, "var_x", x, y, x_add_offset, y_add_offset, x_mult_offset, y_mult_offset);
        emit this->multiTrackingAddItem("ground_truth", x, y);

        x_add_offset += (*s)(0, 0);
        y_add_offset += (*s)(1, 0);
        getPoints(data_values, "var_meas", r, theta);
        polar2Cartesian(r, theta, x, y, x_add_offset, y_add_offset, x_mult_offset, y_mult_offset);
        emit this->multiTrackingAddData("", x, y);

        emit this->multiTrackingAddItem("repaint", x, y);

        if(data_counter >= number_of_steps)
            killTimer(timerId);

    } else if(type_of_tracking == "multi") {

        data_counter += 1;
        QMap<QString, QString> data_values;
        Utils::getDataFromFile(QString("debug_data/MOT/2_1/%1.txt").arg(data_counter), data_values);

        QList<qreal> x, y, r, theta;

        float x_mult_offset = 0.5;
        float y_mult_offset = 0.5;
        int x_add_offset = 1700;
        int y_add_offset = 700;

        getPoints(data_values, "var_gnn", x, y, x_add_offset, y_add_offset, x_mult_offset, y_mult_offset);
        emit this->multiTrackingAddItem("gnn", x, y);

        getPoints(data_values, "var_jpda", x, y, x_add_offset, y_add_offset, x_mult_offset, y_mult_offset);
        emit this->multiTrackingAddItem("jpda", x, y);

        getPoints(data_values, "var_mht", x, y, x_add_offset, y_add_offset, x_mult_offset, y_mult_offset);
        emit this->multiTrackingAddItem("mht", x, y);
        getPoints(data_values, "var_x", x, y, x_add_offset, y_add_offset, x_mult_offset, y_mult_offset);
        emit this->multiTrackingAddItem("ground_truth", x, y);

        x_add_offset += (*s)(0, 0);
        y_add_offset += (*s)(1, 0);
        getPoints(data_values, "var_meas", r, theta);
        polar2Cartesian(r, theta, x, y, x_add_offset, y_add_offset, x_mult_offset, y_mult_offset);
        emit this->multiTrackingAddData("", x, y);

        if(data_counter >= number_of_steps)
            killTimer(timerId);

    } else if(type_of_tracking == "multi_calc") {

        data_counter += 1;
        QList<qreal> x, y, r, theta;

        float x_mult_offset = 0.5;
        float y_mult_offset = 0.5;
        int x_add_offset = 1700;
        int y_add_offset = 1200;

        std::cout << "i: " << data_counter << "--------------------------" << std::endl;
        QMap<QString, QString> data_values;
        Utils::getDataFromFile(QString("debug_data/MOT/%1/%2.txt").arg(data_path, QString::number(data_counter)), data_values);

        MatrixXd z = Utils::getMeasurementData(data_values);

        tracker_jpda->step(z, data_counter==13);
        for(int i = 0; i < nbirths; i++) {
            VectorXd state = tracker_jpda->getX(i);
            x.append(x_mult_offset * (x_add_offset + state(0, 0)));
            y.append(y_mult_offset * (y_add_offset + state(1, 0)));
        }
        emit this->multiTrackingAddItem("jpda", x, y);

        x.clear();
        y.clear();
        tracker_gnn->step(z, data_counter >= 1);
        for(int i = 0; i < nbirths; i++) {
            VectorXd state = tracker_gnn->getX(i);
            x.append(x_mult_offset * (x_add_offset + state(0, 0)));
            y.append(y_mult_offset * (y_add_offset + state(1, 0)));
        }
        emit this->multiTrackingAddItem("gnn", x, y);

        x.clear();
        y.clear();
        tracker_mht->step(z);
        for(int i = 0; i < nbirths; i++) {
            VectorXd state = tracker_mht->getX(i);
            x.append(x_mult_offset * (x_add_offset + state(0, 0)));
            y.append(y_mult_offset * (y_add_offset + state(1, 0)));
        }
        emit this->multiTrackingAddItem("mht", x, y);

        getPoints(data_values, "var_x", x, y) ;//, x_add_offset, y_add_offset, x_mult_offset, y_mult_offset);
        emit this->multiTrackingAddItem("ground_truth", x, y);

        x_add_offset += (*s)(0, 0);
        y_add_offset += (*s)(1, 0);
        getPoints(data_values, "var_meas", r, theta);
        polar2Cartesian(r, theta, x, y, x_add_offset, y_add_offset, x_mult_offset, y_mult_offset);
        emit this->multiTrackingAddData("", x, y);

        emit this->multiTrackingAddItem("repaint", x, y);

        if(data_counter >= number_of_steps)
            killTimer(timerId);
    }
}

void DebugRun::handleEllipsoidalGating() {

    QString path = "2_1_c_200_PD_1";
    QMap<QString, QString> init_values;
    Utils::getDataFromFile(QString("debug_data/SOT/%1/init.txt").arg(path), init_values);
    int K = init_values["K"].toInt();

    shared_ptr<TransitionConstantVelocity> transition_model =
             make_shared<TransitionConstantVelocity>(init_values["T"].toInt(), init_values["sigma_q"].toFloat());

    shared_ptr<MatrixXd> range_c = Utils::getSquareMatrixXdData(init_values, "range_c", 2);
    Sensor sensor(init_values["P_D"].toDouble(), init_values["lambda_c"].toDouble(), *range_c);

    shared_ptr<MeasurementConstantVelocity> measurement_model =
            make_shared<MeasurementConstantVelocity>(init_values["sigma_r"].toFloat());

    int num_errors = 0;
    for(int i = 1; i <= (K - 1); i++) {

        QMap<QString, QString> data_values;
        Utils::getDataFromFile(QString("debug_data/SOT/%1/%2.txt").arg(path, QString::number(i)), data_values);

        shared_ptr<VectorXd> x = Utils::getVectorXdData(data_values, "var_x", 4);
        shared_ptr<MatrixXd> P = Utils::getSquareMatrixXdData(data_values, "var_P", 4);

        State state(x, P);
        Estimator estimator(measurement_model, transition_model);
        MatrixXd z = Utils::getMeasurementData(data_values);

        State post_state = *estimator.predict(state);

        auto result = estimator.ellipsoidalGating(post_state, z, init_values["gating_size"].toFloat());
        ArrayXi meas_in_gate = *std::get<0>(result);
        shared_ptr<MatrixXd> z_ingate = std::get<1>(result);

        QStringList ref_indices = data_values["indices"].split(",");
        for(int j = 0; j < meas_in_gate.size(); j++) {
            int diff = (meas_in_gate(j) + 1) - ref_indices[j].toInt();
            if(diff != 0) {
                num_errors += 1;
                std::cout << "i: " << i << " --> " << (meas_in_gate(j) + 1) << " <> " << ref_indices[j].toStdString() << std::endl;
            }
        }
    }
    std::cout << "Ellipsoidal Gating --> Number of Errors: " << num_errors << std::endl;
}

void DebugRun::handlePredictedLikelihood() {

    QString path = "2_2_c_100_PD_1";
    QMap<QString, QString> init_values;
    Utils::getDataFromFile(QString("debug_data/SOT/%1/init.txt").arg(path), init_values);

    int K = init_values["K"].toInt();
    shared_ptr<TransitionConstantVelocity> transition_model =
             make_shared<TransitionConstantVelocity>(init_values["T"].toInt(), init_values["sigma_q"].toFloat());

    shared_ptr<MatrixXd> range_c = Utils::getSquareMatrixXdData(init_values, "range_c", 2);
    Sensor sensor(init_values["P_D"].toDouble(), init_values["lambda_c"].toDouble(), *range_c);

    shared_ptr<MeasurementConstantVelocity> measurement_model =
            make_shared<MeasurementConstantVelocity>(init_values["sigma_r"].toFloat());

    int num_errors = 0;
    for(int i = 1; i <= K; i++) {

        QMap<QString, QString> data_values;
        Utils::getDataFromFile(QString("debug_data/SOT/%1/%2.txt").arg(path, QString::number(i)), data_values);

        shared_ptr<VectorXd> x = Utils::getVectorXdData(data_values, "var_x", 4);
        shared_ptr<MatrixXd> P = Utils::getSquareMatrixXdData(data_values, "var_P", 4);


        State state(x, P);

        Estimator estimator(measurement_model, transition_model);

        MatrixXd z = Utils::getMeasurementData(data_values);

        shared_ptr<VectorXd> predicted_likelihood = estimator.predictedLikelihood(state, z);

        QStringList ref_values = data_values["predict_likelihood"].split(",");
        for(int j = 0; j < predicted_likelihood->size(); j++) {
            double ref_value = ref_values[j].toDouble();
            double res = predicted_likelihood->coeff(j, 0);
            if(abs(res - ref_value) > 1e-4) {
                num_errors += 1;
                std::cout << "i: " << i << " --> " << ref_value << " <> " << res << std::endl;
            }
        }
    }
    std::cout << "Predicted Likelihood --> Number of Errors: " << num_errors << std::endl;
}

void DebugRun::handleMomentMatching() {

    QString path = "2_3";
    QMap<QString, QString> init_values;
    Utils::getDataFromFile(QString("debug_data/SOT/%1/init.txt").arg(path), init_values);

    int num_gaussians = init_values["numGaussians"].toInt();

    QMap<QString, QString> res_values;
    Utils::getDataFromFile(QString("debug_data/SOT/%1/res.txt").arg(path), res_values);

    shared_ptr<VectorXd> w = Utils::getVectorXdData(res_values, "w", num_gaussians);

    vector<State> states;
    for(int i = 1; i <= num_gaussians; i++) {

        QMap<QString, QString> data_values;
        Utils::getDataFromFile(QString("debug_data/SOT/%1/%2.txt").arg(path, QString::number(i)), data_values);

        shared_ptr<VectorXd> x = Utils::getVectorXdData(data_values, "var_x", 4);
        shared_ptr<MatrixXd> P = Utils::getSquareMatrixXdData(data_values, "var_P", 4);

        states.push_back(State(x, P));
    }

    Estimator estimator;
    State state = estimator.momentMatching(*w, make_shared<vector<State>>(states));

    VectorXd x = *Utils::getVectorXdData(res_values, "var_x", 4);
    MatrixXd P = *Utils::getSquareMatrixXdData(res_values, "var_P", 4);

    // std::cout << state.getX() << std::endl << std::endl;
    // std::cout << x << std::endl << std::endl;

    // std::cout << state.getP() << std::endl << std::endl;
    // std::cout << P << std::endl << std::endl;

    std::cout << "Moment Matching: " << std::endl;
    std::cout << "x norm: " << state.getX().norm() << ", " << x.norm() << std::endl;
    std::cout << "P norm: " << state.getP().norm() << ", " << P.norm() << std::endl;
}

void DebugRun::handlePrune() {

    QString path = "3_1";
    QMap<QString, QString> values;
    Utils::getDataFromFile(QString("debug_data/SOT/%1/values.txt").arg(path), values);

    float threshold = values["threshold"].toFloat();
    vector<int> states(100, 0);

    shared_ptr<VectorXd> weights = Utils::getVectorXdData(values, "hypothesesWeight", 100);

    Hypothesis hypothesis;
    shared_ptr<VectorXd> res_weights;
    shared_ptr<vector<int>> res_states;
    tie(res_weights, res_states) = hypothesis.prune(states, *weights, threshold);

    std::sort(std::begin(*res_weights), std::end(*res_weights));

    int num_errors = 0;
    QStringList ref_weights = values["hypothesesWeight_hat"].split(",");
    for(int i = 0; i < res_weights->rows(); i++) {
        double diff = abs( (*res_weights)(i, 0) -  ref_weights.at(i).toDouble());
        if(diff > 1e-4) {
            num_errors += 1;
            std::cout << diff << std::endl;
        }
    }
    std::cout << "Prune --> Number of Errors: " << num_errors << std::endl;
}

void DebugRun::handleCap() {

    QString path = "3_2";
    QMap<QString, QString> values;
    Utils::getDataFromFile(QString("debug_data/SOT/%1/values.txt").arg(path), values);

    float threshold = values["threshold"].toFloat();
    vector<int> states(100, 0);

    shared_ptr<VectorXd> weights = Utils::getVectorXdData(values, "hypothesesWeight", 100);

    Hypothesis hypothesis;
    shared_ptr<VectorXd> res_weights;
    shared_ptr<vector<int>> res_states;
    tie(res_weights, res_states) = hypothesis.cap(states, *weights, threshold);

    std::sort(std::begin(*res_weights), std::end(*res_weights));

    int num_errors = 0;
    QStringList ref_weights = values["hypothesesWeight_hat"].split(",");
    for(int i = 0; i < res_weights->rows(); i++) {
        double diff = abs( (*res_weights)(i, 0) -  ref_weights.at(i).toDouble());
        if(diff > 1e-4) {
            num_errors += 1;
            std::cout << diff << std::endl;
        }
    }
    std::cout << "Cap --> Number of Errors: " << num_errors << std::endl;
}

void DebugRun::handleNearestNeighbor() {

    QString path = "4_1";
    QMap<QString, QString> init_values;
    Utils::getDataFromFile(QString("debug_data/SOT/%1/init.txt").arg(path), init_values);

    int K = init_values["K"].toInt();
    float P_D = init_values["P_D"].toFloat();
    int T = 1;
    int lambda_c = init_values["lambda_c"].toFloat();
    float sigma_omega = init_values["sigmaOmega"].toFloat();
    float sigma_v = init_values["sigmaV"].toFloat();
    float sigma_b = init_values["sigma_b"].toFloat();
    float sigma_r = init_values["sigma_r"].toFloat();
    float w_min = init_values["wmin"].toFloat();

    shared_ptr<VectorXd> init_x = Utils::getVectorXdData(init_values, "initial_state_x", 5);
    shared_ptr<MatrixXd> init_P = Utils::getSquareMatrixXdData(init_values, "initial_state_P", 5);

    shared_ptr<Vector2d> s = Utils::getVector2dData(init_values, "s", 2);
    shared_ptr<MatrixXd> range_c = Utils::getSquareMatrixXdData(init_values, "range_c", 2);


    shared_ptr<Sensor> sensor = make_shared<Sensor>(P_D, lambda_c, *range_c);
    shared_ptr<Transition2dTurn> transition_model = make_shared<Transition2dTurn>(T, sigma_v, sigma_omega);
    shared_ptr<MeasurementRangeBearing> measurement_model = make_shared<MeasurementRangeBearing>(sigma_r, sigma_b, s);

    shared_ptr<State> init_state = make_shared<State>(init_x, init_P);

    shared_ptr<Estimator> estimator = make_shared<Estimator>(measurement_model, transition_model);
    TrackerNN tracker(estimator, init_state, sensor, GATING_SIZE, w_min);

    int num_errors = 0;
    for(int i = 1; i <= K; i++) {

        QMap<QString, QString> data_values;
        Utils::getDataFromFile(QString("debug_data/SOT/%1/%2.txt").arg(path, QString::number(i)), data_values);

        MatrixXd z = Utils::getMeasurementData(data_values);
        VectorXd ref_x = *Utils::getVectorXdData(data_values, "var_full_x", 5);
        tracker.step(z);
        double diff = abs(ref_x.norm() - tracker.getUpdatedX().norm());

        if(diff > 1e-4) {

            num_errors += 1;
            std::cout << "i: " << i << std::endl;
            std::cout << "result: " << tracker.getUpdatedX() << std::endl;
            std::cout << "ref: " << ref_x << std::endl;
            std::cout << "--------------------------------" << std::endl;
        }
    }
    std::cout << "Nearest Neighbor --> Number of Errors: " << num_errors << std::endl;
}

void DebugRun::handlePDA() {

    QString path = "4_2";
    QMap<QString, QString> init_values;
    Utils::getDataFromFile(QString("debug_data/SOT/%1/init.txt").arg(path), init_values);

    int K = init_values["K"].toInt();
    float P_D = init_values["P_D"].toFloat();
    int T = 1;
    int lambda_c = init_values["lambda_c"].toFloat();
    float sigma_omega = init_values["sigmaOmega"].toFloat();
    float sigma_v = init_values["sigmaV"].toFloat();
    float sigma_b = init_values["sigma_b"].toFloat();
    float sigma_r = init_values["sigma_r"].toFloat();
    float w_min = init_values["wmin"].toFloat();

    shared_ptr<VectorXd> init_x = Utils::getVectorXdData(init_values, "initial_state_x", 5);
    shared_ptr<MatrixXd> init_P = Utils::getSquareMatrixXdData(init_values, "initial_state_P", 5);

    shared_ptr<Vector2d> s = Utils::getVector2dData(init_values, "s", 2);
    shared_ptr<MatrixXd> range_c = Utils::getSquareMatrixXdData(init_values, "range_c", 2);

    shared_ptr<Sensor> sensor = make_shared<Sensor>(P_D, lambda_c, *range_c);
    shared_ptr<Transition2dTurn> transition_model = make_shared<Transition2dTurn>(T, sigma_v, sigma_omega);
    shared_ptr<MeasurementRangeBearing> measurement_model = make_shared<MeasurementRangeBearing>(sigma_r, sigma_b, s);

    shared_ptr<State> init_state = make_shared<State>(init_x, init_P);

    shared_ptr<Estimator> estimator = make_shared<Estimator>(measurement_model, transition_model);
    TrackerPDA tracker(estimator, init_state, sensor, GATING_SIZE, w_min);

    int num_errors = 0;
    for(int i = 1; i <= K; i++) {

        QMap<QString, QString> data_values;
        Utils::getDataFromFile(QString("debug_data/SOT/%1/%2.txt").arg(path, QString::number(i)), data_values);

        MatrixXd z = Utils::getMeasurementData(data_values);
        VectorXd ref_x = *Utils::getVectorXdData(data_values, "var_full_x", 5);
        tracker.step(z);

        double diff = abs(ref_x.norm() - tracker.getUpdatedX().norm());

        if(diff > 1e-4) {

            num_errors += 1;
            std::cout << "i: " << i << std::endl;
            std::cout << "result: " << tracker.getUpdatedX() << std::endl;
            std::cout << "ref: " << ref_x << std::endl;
            std::cout << "diff: " << (tracker.getUpdatedX() - ref_x) << std::endl;
            std::cout << "--------------------------------" << std::endl;
        }
    }
    std::cout << "PDA --> Number of Errors: " << num_errors << std::endl;
}

void DebugRun::handleGaussianSum() {

    QString path = "4_3";
    QMap<QString, QString> init_values;
    Utils::getDataFromFile(QString("debug_data/SOT/%1/init.txt").arg(path), init_values);

    int K = init_values["K"].toInt();
    int M = init_values["M"].toInt();
    float P_D = init_values["P_D"].toFloat();
    int T = init_values["T"].toInt();
    int lambda_c = init_values["lambda_c"].toFloat();
    float sigma_omega = init_values["sigmaOmega"].toFloat();
    float sigma_v = init_values["sigmaV"].toFloat();
    float sigma_b = init_values["sigma_b"].toFloat();
    float sigma_r = init_values["sigma_r"].toFloat();
    float w_min = init_values["wmin"].toFloat();

    shared_ptr<VectorXd> init_x = Utils::getVectorXdData(init_values, "initial_state_x", 5);
    shared_ptr<MatrixXd> init_P = Utils::getSquareMatrixXdData(init_values, "initial_state_P", 5);

    shared_ptr<Vector2d> s = Utils::getVector2dData(init_values, "s", 2);
    shared_ptr<MatrixXd> range_c = Utils::getSquareMatrixXdData(init_values, "range_c", 2);

    shared_ptr<Sensor> sensor = make_shared<Sensor>(P_D, lambda_c, *range_c);
    shared_ptr<Transition2dTurn> transition_model = make_shared<Transition2dTurn>(T, sigma_v, sigma_omega);
    shared_ptr<MeasurementRangeBearing> measurement_model = make_shared<MeasurementRangeBearing>(sigma_r, sigma_b, s);

    shared_ptr<State> init_state = make_shared<State>(init_x, init_P);

    shared_ptr<Estimator> estimator = make_shared<Estimator>(measurement_model, transition_model);
    TrackerGaussianSum tracker(estimator, init_state, sensor, GATING_SIZE, M, w_min);

    int num_errors = 0;
    for(int i = 1; i <= K; i++) {

        QMap<QString, QString> data_values;
        Utils::getDataFromFile(QString("debug_data/SOT/%1/%2.txt").arg(path, QString::number(i)), data_values);

        MatrixXd z = Utils::getMeasurementData(data_values);
        VectorXd ref_x = *Utils::getVectorXdData(data_values, "var_full_x", 5);
        tracker.step(z);

        double diff = abs(ref_x.norm() - tracker.getUpdatedX().norm());
        if(diff > 1e-4) {

            num_errors += 1;
            std::cout << "i: " << i << std::endl;
            std::cout << "result: " << tracker.getUpdatedX() << std::endl;
            std::cout << "ref: " << ref_x << std::endl;
            std::cout << "diff: " << (tracker.getUpdatedX() - ref_x) << std::endl;
            std::cout << "--------------------------------" << std::endl;
        }
    }
    std::cout << "Gaussian Sum --> Number of Errors: " << num_errors << std::endl;
}

void DebugRun::handleGNN() {

    QString path = "1_1";
    QMap<QString, QString> init_values;
    Utils::getDataFromFile(QString("debug_data/MOT/%1/init.txt").arg(path), init_values);

    qDebug() << init_values;

    int K = init_values["K"].toInt();
    number_of_steps = K;
    int M = init_values["M"].toInt();
    float P_D = init_values["P_D"].toFloat();
//    float P_G = init_values["P_G"].toFloat();
    int T = init_values["T"].toInt();
    int lambda_c = init_values["lambda_c"].toFloat();
    // int mergeing_threshold = init_values["merging_threshold"].toInt();
    nbirths = init_values["nbirths"].toInt();
    float sigma_omega = init_values["sigmaOmega"].toFloat();
    float sigma_v = init_values["sigmaV"].toFloat();
    float sigma_b = init_values["sigma_b"].toFloat();
    float sigma_r = init_values["sigma_r"].toFloat();
    float w_min = init_values["w_min"].toFloat();

    s = Utils::getVector2dData(init_values, "s", 2);
    shared_ptr<MatrixXd> range_c = Utils::getSquareMatrixXdData(init_values, "range_c", 2);

    shared_ptr<Sensor> sensor = make_shared<Sensor>(P_D, lambda_c, *range_c);
    shared_ptr<Transition2dTurn> transition_model = make_shared<Transition2dTurn>(T, sigma_v, sigma_omega);
    shared_ptr<MeasurementRangeBearing> measurement_model = make_shared<MeasurementRangeBearing>(sigma_r, sigma_b, s);

    PtrVecState states = make_shared<vector<shared_ptr<State>>>();

    for(int i = 1; i <= 4; i++) {

        QMap<QString, QString> state_values;
        Utils::getDataFromFile(QString("debug_data/MOT/%1/init_states_%2.txt").arg(path, QString::number(i)), state_values);

        shared_ptr<VectorXd> x = Utils::getVectorXdData(state_values, "var_x", 5);
        shared_ptr<MatrixXd> P = Utils::getSquareMatrixXdData(state_values, "var_P", 5);

        states->push_back(make_shared<State>(x, P));
    }

    shared_ptr<Estimator> estimator = make_shared<Estimator>(measurement_model, transition_model);
    tracker_gnn = make_shared<MultiTrackerGNN>(estimator, states, sensor, GATING_SIZE, M, w_min);


//    for(int i = 1; i <= K; i++) {

//        QMap<QString, QString> data_values;
//        Utils::getDataFromFile(QString("debug_data/MOT/%1/%2.txt").arg(path, QString::number(i)), data_values);

//        if(i == 13) {
//            qDebug() << "Start of Debugging ...";
//        }
//        std::cout << "i: " << i << "--------------------------" << std::endl;
//        MatrixXd z = Utils::getMeasurementData(data_values);
//        tracker_gnn->step(z);
//    }
}

void DebugRun::handleJPDA() {

    QString path = "2_1";
    QMap<QString, QString> init_values;
    Utils::getDataFromFile(QString("debug_data/MOT/%1/init.txt").arg(path), init_values);

    qDebug() << init_values;

    int K = init_values["K"].toInt();
    number_of_steps = K;
    int M = init_values["M"].toInt();
    float P_D = init_values["P_D"].toFloat();
//    float P_G = init_values["P_G"].toFloat();
    int T = init_values["T"].toInt();
    int lambda_c = init_values["lambda_c"].toFloat();
//    int mergeing_threshold = init_values["merging_threshold"].toInt();
    nbirths = init_values["nbirths"].toInt();
    float sigma_omega = init_values["sigmaOmega"].toFloat();
    float sigma_v = init_values["sigmaV"].toFloat();
    float sigma_b = init_values["sigma_b"].toFloat();
    float sigma_r = init_values["sigma_r"].toFloat();
    float w_min = init_values["w_min"].toFloat();

    s = Utils::getVector2dData(init_values, "s", 2);
    shared_ptr<MatrixXd> range_c = Utils::getSquareMatrixXdData(init_values, "range_c", 2);

    shared_ptr<Sensor> sensor = make_shared<Sensor>(P_D, lambda_c, *range_c);
    shared_ptr<Transition2dTurn> transition_model = make_shared<Transition2dTurn>(T, sigma_v, sigma_omega);
    shared_ptr<MeasurementRangeBearing> measurement_model = make_shared<MeasurementRangeBearing>(sigma_r, sigma_b, s);

    PtrVecState states = make_shared<vector<shared_ptr<State>>>();

    for(int i = 1; i <= 4; i++) {

        QMap<QString, QString> state_values;
        Utils::getDataFromFile(QString("debug_data/MOT/%1/init_states_%2.txt").arg(path, QString::number(i)), state_values);

        shared_ptr<VectorXd> x = Utils::getVectorXdData(state_values, "var_x", 5);
        shared_ptr<MatrixXd> P = Utils::getSquareMatrixXdData(state_values, "var_P", 5);

        states->push_back(make_shared<State>(x, P));
    }

    shared_ptr<Estimator> estimator = make_shared<Estimator>(measurement_model, transition_model);
    tracker_jpda = make_shared<MultiTrackerJPDA>(estimator, states, sensor, GATING_SIZE, M, w_min);

//    for(int i = 1; i <= K; i++) {

//        QMap<QString, QString> data_values;
//        Utils::getDataFromFile(QString("debug_data/MOT/%1/%2.txt").arg(path, QString::number(i)), data_values);

//        std::cout << "i: " << i << "--------------------------" << std::endl;
//        MatrixXd z = Utils::getMeasurementData(data_values);
//        tracker_jpda->step(z);
//    }
}

void DebugRun::handleMHT() {

    QString path = "3_1_c_10_PD_9";
    QMap<QString, QString> init_values;
    Utils::getDataFromFile(QString("debug_data/MOT/%1/init.txt").arg(path), init_values);

    qDebug() << init_values;

    int K = init_values["K"].toInt();
    int M = init_values["M"].toInt();
    float P_D = init_values["P_D"].toFloat();
    // float P_G = init_values["P_G"].toFloat();
    int T = init_values["T"].toInt();
    int lambda_c = init_values["lambda_c"].toFloat();
//    int mergeing_threshold = init_values["merging_threshold"].toInt();
    int nbirths = init_values["nbirths"].toInt();
    float sigma_omega = init_values["sigmaOmega"].toFloat();
    float sigma_v = init_values["sigmaV"].toFloat();
    float sigma_b = init_values["sigma_b"].toFloat();
    float sigma_r = init_values["sigma_r"].toFloat();
    float w_min = init_values["w_min"].toFloat();

    shared_ptr<Vector2d> s = Utils::getVector2dData(init_values, "s", 2);
    shared_ptr<MatrixXd> range_c = Utils::getSquareMatrixXdData(init_values, "range_c", 2);

    shared_ptr<Sensor> sensor = make_shared<Sensor>(P_D, lambda_c, *range_c);
    shared_ptr<Transition2dTurn> transition_model = make_shared<Transition2dTurn>(T, sigma_v, sigma_omega);
    shared_ptr<MeasurementRangeBearing> measurement_model = make_shared<MeasurementRangeBearing>(sigma_r, sigma_b, s);

    PtrVecState states = make_shared<vector<shared_ptr<State>>>();

    for(int i = 1; i <= nbirths; i++) {

        QMap<QString, QString> state_values;
        Utils::getDataFromFile(QString("debug_data/MOT/%1/init_states_%2.txt").arg(path, QString::number(i)), state_values);

        shared_ptr<VectorXd> x = Utils::getVectorXdData(state_values, "var_x", 5);
        shared_ptr<MatrixXd> P = Utils::getSquareMatrixXdData(state_values, "var_P", 5);

        states->push_back(make_shared<State>(x, P));
    }

    shared_ptr<Estimator> estimator = make_shared<Estimator>(measurement_model, transition_model);
    MultiTrackerMHT tracker(estimator, states, sensor, 13.8155, M, w_min);
    tracker.enablePrintResult();

    for(int i = 1; i <= K; i++) {

        QMap<QString, QString> data_values;
        Utils::getDataFromFile(QString("debug_data/MOT/%1/%2.txt").arg(path, QString::number(i)), data_values);

        std::cout << "Tacker index: " << i << "--------------------------" << std::endl;
        MatrixXd z = Utils::getMeasurementData(data_values);
        tracker.step(z, i == 5);
    }
}

void DebugRun::handleIndices() {

    QString path = "mht_index";
    QMap<QString, QString> init_values;

    for(int k = 1; k <= 6; k++) {

        std::cout << "k: " << k << "------------------------" << std::endl;
        Utils::getDataFromFile(QString("debug_data/MOT/%1/indices_%2.txt").arg(path, QString::number(k)), init_values);
        int n = init_values["n"].toInt();
        // qDebug() << init_values;
        MatrixXd pre_z = Utils::getMeasurementData(init_values, "pre_z");
        MatrixXd z = Utils::getMeasurementData(init_values, "z");
        // std::cout << z << std::endl;
        // qDebug () << z.cols();
        pair<int, int> LEN = Utils::getLen(init_values, "sz_pre_idx_z_ingate");
        // qDebug() << LEN.first << ", " << LEN.second;
        MatrixXd pre_idx_z = *Utils::getRectangleMatrixXdData(init_values, "pre_idx_z_ingate", LEN.first, LEN.second);
        // std::cout << pre_idx_z << std::endl;
        std::set<int> set_gated_index;
        vector<vector<ArrayXi>> gated_index;


        int j = 0;
        for(int i = 0; i < n; i++) {
            QMap<QString, QString> new_indices_values;
            Utils::getDataFromFile(QString("debug_data/MOT/%1/new_indices_%2_%3.txt").arg(
                                       path, QString::number(k), QString::number(i+1)), new_indices_values);
            int n_i = new_indices_values["n_i"].toInt();

            vector<ArrayXi> inner_gated_index;
            for(int lh = 0; lh < n_i; lh++) {
                int arr_len = (pre_idx_z(Eigen::all, j).array() > 0).colwise().count()[0];
                ArrayXi curr_gated_index(arr_len);

                int l = 0;
                for(int k = 0; k < pre_idx_z.rows(); k++) {
                    if(pre_idx_z(k, j) != 0) {
                        set_gated_index.insert(k);
                        curr_gated_index(l) = k;
                        l += 1;
                    }
                }
                inner_gated_index.push_back(curr_gated_index);
                j += 1;
            }
            gated_index.push_back(inner_gated_index);
        }


//        for(int j = 0; j < pre_idx_z.cols(); j++) {
//            int l = 0;
//            for(int k = 0; k < pre_idx_z.rows(); k++) {
//                if(pre_idx_z(k, j) != 0) {
//                    set_gated_index.insert(k);
//                }
//            }
//        }
//        for (auto const& index : set_gated_index)
//            std::cout << index << " ";
//        std::cout << std::endl << "-----------------------------" << std::endl;
        int m = z.cols();
        vector<int> all_indices(m);
        std::generate(all_indices.begin(), all_indices.end(), [n = 0]() mutable { return n++; });
        set<int> set_all_indices(all_indices.begin(), all_indices.end());
        std::set<int> set_clutter_indices;
        std::set_difference(set_all_indices.begin(), set_all_indices.end(),
                            set_gated_index.begin(), set_gated_index.end(),
                            std::inserter(set_clutter_indices, set_clutter_indices.end()));

//        for (auto const& index : set_clutter_indices)
//            std::cout << index << " ";
//        std::cout << std::endl;
        vector<int> vec_gated_index(set_gated_index.begin(), set_gated_index.end());
        sort(vec_gated_index.begin(), vec_gated_index.end());
        std::map<int, int> L_indices;
        for(int i = 0; i < set_gated_index.size(); i++) {
            L_indices[vec_gated_index[i]] = i;
        }

        for(int i = 0; i < n; i++) {
            std::cout << "############" << std::endl;
            // std::cout << "n: " << n << std::endl;
            // int n_i = this->H_i->at(i)->size();
            QMap<QString, QString> new_indices_values;
            Utils::getDataFromFile(QString("debug_data/MOT/%1/new_indices_%2_%3.txt").arg(
                                       path, QString::number(k), QString::number(i+1)), new_indices_values);
            int n_i = new_indices_values["n_i"].toInt();
            // std::cout << "n_i: " << n_i << std::endl;

            std::cout << "newidx: \t";
            for(int lh = 0; lh < n_i; lh++) {

                for(int p = 0; p < gated_index[i][lh].rows(); p++) {
                    int j = gated_index[i][lh](p, 0);
                    // std::cout << "j: " << j << std::endl;
                    int L_idx = L_indices[j];
                    // std::cout << "L_idx: " << L_idx << std::endl;
                    int newidx = (lh)*(m+1) + L_idx;
                    std::cout << (newidx + 1) << " ";

                }
                int newidx = (lh+1)*(m+1)-1;
                std::cout << (newidx + 1) << " ";
            }
            std::cout << std::endl;
            std::cout << "ref file: \t" << new_indices_values["new_indices"].toStdString() << std::endl;
        }


    }

}






