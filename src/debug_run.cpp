#include "inc/debug_run.h"

// [✓] - Read data from res.txt File
// [✓] - Define 4 arrays in Javascript for 'NearestNeighbor, PDA, GaussianSum, GroundTruth'
// [✓] - Show Measurements in UI
// [ ] - Define function to initialize Trackers
// [ ] - Define initialization variables as Class Members
// [ ] - Write function in MultiTracker to getX by index of Object
// [ ] - Add for-loop to 'initTrackers' to get result of 'tracker_mht' and compare to result in text file
// [ ] - Show result of 'tracker_gnn' in UI


DebugRun::DebugRun(const QQmlApplicationEngine &engine, QObject *parent) : QObject(parent) {

    // engine.rootContext()->setContextProperty("backend", &connectionHandler);
    engine.rootContext()->setContextProperty("backend", this);

    data_path = "2_2";
    initTrackers();
}

void DebugRun::initTrackers() {

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
//    handleGNN();
//    handleJPDA();
//    handleMHT();
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

        QMap<QString, QString> res_values;
        Utils::getDataFromFile("debug_data/SOT/sim/res.txt", res_values);

        data_counter = 0;

        float x_mult_offset = 2;
        int y_add_offset = 50;
        getPoints(res_values, "NN_estimated_state", x_nn, y_nn, 0, y_add_offset, x_mult_offset, 1);
        getPoints(res_values, "GS_estimated_state", x_pda, y_pda, 0, y_add_offset, x_mult_offset, 1);
        getPoints(res_values, "PDA_estimated_state", x_gs, y_gs, 0, y_add_offset, x_mult_offset, 1);
        getPoints(res_values, "true_state", x_ground, y_ground, 0, y_add_offset, x_mult_offset, 1);

        timerId = startTimer(100);

    } else if(type_of_tracking == "multi") {

        QMap<QString, QString> init_values;
        Utils::getDataFromFile("debug_data/MOT/sim/init.txt", init_values);

        nbirths = init_values["nbirths"].toInt();
        number_of_steps = init_values["K"].toInt();
        s = Utils::getVector2dData(init_values, "s", 2);
        emit setNumberOfBirth(nbirths);
        data_counter = 0;
        timerId = startTimer(50);

    } else if(type_of_tracking == "jpda" || type_of_tracking == "gnn") {

        emit setNumberOfBirth(nbirths);
        data_counter = 0;
        timerId = startTimer(50);

    } else if(type_of_tracking == "stop") {
        killTimer(timerId);
    }
}

void DebugRun::timerEvent(QTimerEvent *event) {

    if(type_of_tracking == "single") {

        emit this->singleTrackingAddItem("nearest_neighbor", x_nn[data_counter], y_nn[data_counter]);
        emit this->singleTrackingAddItem("gaussian_sum", x_pda[data_counter], y_pda[data_counter]);
        emit this->singleTrackingAddItem("pda", x_gs[data_counter], y_gs[data_counter]);
        emit this->singleTrackingAddItem("true_state", x_ground[data_counter], y_ground[data_counter]);

        data_counter += 1;
        if(data_counter >= x_nn.size())
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

    } else if(type_of_tracking == "jpda") {

        data_counter += 1;
        QList<qreal> x, y, r, theta;

        float x_mult_offset = 0.5;
        float y_mult_offset = 0.5;
        int x_add_offset = 1700;
        int y_add_offset = 1200;

        QMap<QString, QString> data_values;
        Utils::getDataFromFile(QString("debug_data/MOT/%1/%2.txt").arg(data_path, QString::number(data_counter)), data_values);

        MatrixXd z = Utils::getMeasurementData(data_values);

        tracker_jpda->step(z);
        for(int i = 0; i < nbirths; i++) {
            VectorXd state = tracker_jpda->getX(i);
            x.append(x_mult_offset * (x_add_offset + state(0, 0)));
            y.append(y_mult_offset * (y_add_offset + state(1, 0)));
        }
        emit this->multiTrackingAddItem("jpda", x, y);

        x.clear();
        y.clear();
        tracker_gnn->step(z);
        for(int i = 0; i < nbirths; i++) {
            VectorXd state = tracker_gnn->getX(i);
            x.append(x_mult_offset * (x_add_offset + state(0, 0)));
            y.append(y_mult_offset * (y_add_offset + state(1, 0)));
        }
        emit this->multiTrackingAddItem("gnn", x, y);

//        x.clear();
//        y.clear();
//        tracker_mht->step(z);
//        for(int i = 0; i < nbirths; i++) {
//            VectorXd state = tracker_mht->getX(i);
//            x.append(x_mult_offset * (x_add_offset + state(0, 0)));
//            y.append(y_mult_offset * (y_add_offset + state(1, 0)));
//        }
//        emit this->multiTrackingAddItem("mht", x, y);


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

    } else if(type_of_tracking == "gnn") {

        data_counter += 1;
        QList<qreal> x, y, r, theta;

        float x_mult_offset = 0.5;
        float y_mult_offset = 0.5;
        int x_add_offset = 1700;
        int y_add_offset = 1200;

        QMap<QString, QString> data_values;
        Utils::getDataFromFile(QString("debug_data/MOT/%1/%2.txt").arg("1_1", QString::number(data_counter)), data_values);

        qDebug() << "i: " << data_counter << "-------------------";
        MatrixXd z = Utils::getMeasurementData(data_values);
        tracker_gnn->step(z);

        for(int i = 0; i < nbirths; i++) {
            VectorXd state = tracker_gnn->getX(i);
            x.append(x_mult_offset * (x_add_offset + state(0, 0)));
            y.append(y_mult_offset * (y_add_offset + state(1, 0)));
        }

        emit this->multiTrackingAddItem("gnn", x, y);

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
    }
}

void DebugRun::handleEllipsoidalGating() {

    QMap<QString, QString> init_values;
    Utils::getDataFromFile("debug_data/SOT/2_1/init.txt", init_values);

    shared_ptr<TransitionConstantVelocity> transition_model =
             make_shared<TransitionConstantVelocity>(init_values["T"].toInt(), init_values["sigma_q"].toFloat());

    shared_ptr<MatrixXd> range_c = Utils::getSquareMatrixXdData(init_values, "range_c", 2);
    Sensor sensor(init_values["P_D"].toDouble(), init_values["lambda_c"].toDouble(), *range_c);

    shared_ptr<MeasurementConstantVelocity> measurement_model =
            make_shared<MeasurementConstantVelocity>(init_values["sigma_r"].toFloat());

    for(int i = 1; i <= 9; i++) {

        qDebug() << i << "-------------------------------";

        QMap<QString, QString> data_values;
        Utils::getDataFromFile(QString("debug_data/SOT/2_1/%1.txt").arg(i), data_values);

        shared_ptr<VectorXd> x = Utils::getVectorXdData(data_values, "var_x", 4);
        shared_ptr<MatrixXd> P = Utils::getSquareMatrixXdData(data_values, "var_P", 4);

        State state(x, P);

        Estimator estimator(measurement_model, transition_model);

        MatrixXd z = Utils::getMeasurementData(data_values);

        auto result = estimator.ellipsoidalGating(state, z, init_values["gating_size"].toFloat());
        shared_ptr<ArrayXi> meas_in_gate = std::get<0>(result);
        shared_ptr<MatrixXd> z_ingate = std::get<1>(result);

        qDebug() << data_values["indices"];
        for(int i = 0; i < meas_in_gate->size(); i++)
            // qDebug() << (*z_ingate)(0, i) << " " << (*z_ingate)(1, i);
            qDebug() << (*meas_in_gate)(i);
    }
}

void DebugRun::handlePredictedLikelihood() {

    QMap<QString, QString> init_values;
    Utils::getDataFromFile("debug_data/SOT/2_2/init.txt", init_values);

    shared_ptr<TransitionConstantVelocity> transition_model =
             make_shared<TransitionConstantVelocity>(init_values["T"].toInt(), init_values["sigma_q"].toFloat());

    shared_ptr<MatrixXd> range_c = Utils::getSquareMatrixXdData(init_values, "range_c", 2);
    Sensor sensor(init_values["P_D"].toDouble(), init_values["lambda_c"].toDouble(), *range_c);

    shared_ptr<MeasurementConstantVelocity> measurement_model =
            make_shared<MeasurementConstantVelocity>(init_values["sigma_r"].toFloat());

    for(int i = 1; i <= 9; i++) {

        qDebug() << i << "-------------------------------";

        QMap<QString, QString> data_values;
        Utils::getDataFromFile(QString("debug_data/SOT/2_2/%1.txt").arg(i), data_values);

        shared_ptr<VectorXd> x = Utils::getVectorXdData(data_values, "var_x", 4);
        shared_ptr<MatrixXd> P = Utils::getSquareMatrixXdData(data_values, "var_P", 4);


        State state(x, P);

        Estimator estimator(measurement_model, transition_model);

        MatrixXd z = Utils::getMeasurementData(data_values);

        shared_ptr<VectorXd> predicted_likelihood = estimator.predictedLikelihood(state, z);

        qDebug() << data_values["predict_likelihood"];
        for(int i = 0; i < predicted_likelihood->size(); i++)
            qDebug() << predicted_likelihood->coeff(i, 0);
    }
}

void DebugRun::handleMomentMatching() {

    QMap<QString, QString> init_values;
    Utils::getDataFromFile("debug_data/SOT/2_3/init.txt", init_values);

    int num_gaussians = init_values["numGaussians"].toInt();
    int num_dims = init_values["nDim"].toInt();

    QMap<QString, QString> res_values;
    Utils::getDataFromFile("debug_data/SOT/2_3/res.txt", res_values);

    shared_ptr<VectorXd> w = Utils::getVectorXdData(res_values, "w", num_gaussians);

    vector<State> states;
    for(int i = 1; i <= num_gaussians; i++) {

        QMap<QString, QString> data_values;
        Utils::getDataFromFile(QString("debug_data/SOT/2_3/%1.txt").arg(i), data_values);

        shared_ptr<VectorXd> x = Utils::getVectorXdData(data_values, "var_x", 4);
        shared_ptr<MatrixXd> P = Utils::getSquareMatrixXdData(data_values, "var_P", 4);

        states.push_back(State(x, P));
    }

    Estimator estimator;
    State state = estimator.momentMatching(*w, make_shared<vector<State>>(states));

    std::cout << state.getX() << std::endl << std::endl;
    std::cout << state.getP() << std::endl << std::endl;
    std::cout << res_values["var_x"].toStdString() << std::endl << std::endl;
    std::cout << res_values["var_P"].toStdString() << std::endl << std::endl;
}

void DebugRun::handlePrune() {

    QMap<QString, QString> values;
    Utils::getDataFromFile("debug_data/SOT/3_1/values.txt", values);

    float threshold = values["threshold"].toFloat();
    vector<int> states(100, 0);

    shared_ptr<VectorXd> weights = Utils::getVectorXdData(values, "hypothesesWeight", 100);

    Hypothesis hypothesis;
    shared_ptr<VectorXd> res_weights;
    shared_ptr<vector<int>> res_states;
    tie(res_weights, res_states) = hypothesis.prune(states, *weights, threshold);

    std::sort(std::begin(*res_weights), std::end(*res_weights));

    std::cout << values["hypothesesWeight_hat"].toStdString() << std::endl << std::endl;
    std::cout << res_weights->rows() << std::endl << std::endl;
    std::cout << (*res_weights) << std::endl;

}

void DebugRun::handleCap() {

    QMap<QString, QString> values;
    Utils::getDataFromFile("debug_data/SOT/3_2/values.txt", values);

    float threshold = values["threshold"].toFloat();
    vector<int> states(100, 0);

    shared_ptr<VectorXd> weights = Utils::getVectorXdData(values, "hypothesesWeight", 100);

    Hypothesis hypothesis;
    shared_ptr<VectorXd> res_weights;
    shared_ptr<vector<int>> res_states;
    tie(res_weights, res_states) = hypothesis.cap(states, *weights, threshold);

    std::sort(std::begin(*res_weights), std::end(*res_weights));

    std::cout << values["hypothesesWeight_hat"].toStdString() << std::endl << std::endl;
    std::cout << res_weights->rows() << std::endl << std::endl;
    std::cout << (*res_weights) << std::endl;
}

void DebugRun::handleNearestNeighbor() {

    QMap<QString, QString> init_values;
    Utils::getDataFromFile("debug_data/SOT/4_1/init.txt", init_values);

    qDebug() << init_values;

    int K = init_values["K"].toInt();
    int M = init_values["M"].toInt();
    float P_D = init_values["P_D"].toFloat();
    float P_G = init_values["P_G"].toFloat();
    int T = 1;
    int lambda_c = init_values["lambda_c"].toFloat();
    int mergeing_threshold = init_values["merging_threshold"].toInt();
    int nbirths = init_values["nbirths"].toInt();
    float sigma_omega = init_values["sigmaOmega"].toFloat();
    float sigma_v = init_values["sigmaV"].toFloat();
    float sigma_b = init_values["sigma_b"].toFloat();
    float sigma_r = init_values["sigma_r"].toFloat();
    float w_min = init_values["wmin"].toFloat();

    shared_ptr<VectorXd> init_x = Utils::getVectorXdData(init_values, "initial_state_x", 5);
    shared_ptr<MatrixXd> init_P = Utils::getSquareMatrixXdData(init_values, "initial_state_P", 5);

    shared_ptr<Vector2d> s = Utils::getVector2dData(init_values, "s", 2);
    shared_ptr<MatrixXd> range_c = Utils::getSquareMatrixXdData(init_values, "range_c", 2);

//    std::cout << *init_x << std::endl << std::endl;
//    std::cout << *init_P << std::endl << std::endl;
//    std::cout << *s << std::endl << std::endl;
//    std::cout << *range_c << std::endl << std::endl;

    shared_ptr<Sensor> sensor = make_shared<Sensor>(P_D, lambda_c, *range_c);
    shared_ptr<Transition2dTurn> transition_model = make_shared<Transition2dTurn>(T, sigma_v, sigma_omega);
    shared_ptr<MeasurementRangeBearing> measurement_model = make_shared<MeasurementRangeBearing>(sigma_r, sigma_b, s);

    shared_ptr<State> init_state = make_shared<State>(init_x, init_P);

    shared_ptr<Estimator> estimator = make_shared<Estimator>(measurement_model, transition_model);
    TrackerNN tracker(estimator, init_state, sensor, 13.8155, w_min);

    for(int i = 1; i <= 50; i++) {

        QMap<QString, QString> data_values;
        Utils::getDataFromFile(QString("debug_data/SOT/4_1/%1.txt").arg(i), data_values);

        std::cout << i << std::endl;
        MatrixXd z = Utils::getMeasurementData(data_values);
        tracker.step(z);
        // std::cout << tracker.getX() << std::endl << std::endl;
    }
}

void DebugRun::handlePDA() {

    QMap<QString, QString> init_values;
    Utils::getDataFromFile("debug_data/SOT/4_2/init.txt", init_values);

    qDebug() << init_values;

    int K = init_values["K"].toInt();
    int M = init_values["M"].toInt();
    float P_D = init_values["P_D"].toFloat();
    float P_G = init_values["P_G"].toFloat();
    int T = 1;
    int lambda_c = init_values["lambda_c"].toFloat();
    int mergeing_threshold = init_values["merging_threshold"].toInt();
    int nbirths = init_values["nbirths"].toInt();
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
    TrackerPDA tracker(estimator, init_state, sensor, 13.8155, w_min);

    for(int i = 1; i <= 50; i++) {

        QMap<QString, QString> data_values;
        Utils::getDataFromFile(QString("debug_data/SOT/4_2/%1.txt").arg(i), data_values);

        std::cout << i << std::endl;
        MatrixXd z = Utils::getMeasurementData(data_values);
        tracker.step(z);
    }
}

void DebugRun::handleGaussianSum() {

    QMap<QString, QString> init_values;
    Utils::getDataFromFile("debug_data/SOT/4_3/init.txt", init_values);

    qDebug() << init_values;

    int K = init_values["K"].toInt();
    int M = init_values["M"].toInt();
    float P_D = init_values["P_D"].toFloat();
    float P_G = init_values["P_G"].toFloat();
    int T = init_values["T"].toInt(); // 1;
    int lambda_c = init_values["lambda_c"].toFloat();
    int mergeing_threshold = init_values["merging_threshold"].toInt();
    int nbirths = init_values["nbirths"].toInt();
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
    // TrackerNN tracker(estimator, init_state, sensor, 13.8155, w_min);
    TrackerGaussianSum tracker(estimator, init_state, sensor, 13.8155, M, w_min);

    for(int i = 1; i <= 50; i++) {

        QMap<QString, QString> data_values;
        Utils::getDataFromFile(QString("debug_data/SOT/4_3/%1.txt").arg(i), data_values);

        std::cout << "i: " << i << std::endl;
        MatrixXd z = Utils::getMeasurementData(data_values);
        tracker.step(z);
    }
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

    QString path = "3_1";
    QMap<QString, QString> init_values;
    Utils::getDataFromFile(QString("debug_data/MOT/%1/init.txt").arg(path), init_values);

    qDebug() << init_values;

    int K = init_values["K"].toInt();
    int M = init_values["M"].toInt();
    float P_D = init_values["P_D"].toFloat();
    float P_G = init_values["P_G"].toFloat();
    int T = init_values["T"].toInt();
    int lambda_c = init_values["lambda_c"].toFloat();
    int mergeing_threshold = init_values["merging_threshold"].toInt();
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

    for(int i = 1; i <= 4; i++) {

        QMap<QString, QString> state_values;
        Utils::getDataFromFile(QString("debug_data/MOT/%1/init_states_%2.txt").arg(path, QString::number(i)), state_values);

        shared_ptr<VectorXd> x = Utils::getVectorXdData(state_values, "var_x", 5);
        shared_ptr<MatrixXd> P = Utils::getSquareMatrixXdData(state_values, "var_P", 5);

        states->push_back(make_shared<State>(x, P));
    }

    shared_ptr<Estimator> estimator = make_shared<Estimator>(measurement_model, transition_model);
    MultiTrackerMHT tracker(estimator, states, sensor, 13.8155, M, w_min);

//    for(int i = 1; i <= 20; i++) {

//        QMap<QString, QString> data_values;
//        Utils::getDataFromFile(QString("debug_data/MOT/%1/%2.txt").arg(path, QString::number(i)), data_values);

//        std::cout << "i: " << i << "--------------------------" << std::endl;
//        MatrixXd z = Utils::getMeasurementData(data_values);
//        tracker.step(z);
//    }
}


