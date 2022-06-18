
#include <QCoreApplication>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickView>
#include <QQmlContext>
#include "inc/debug_server.h"
#include "ui/inc/ui_connection_handler.h"


void handleEllipsoidalGating();
void handlePredictedLikelihood();
void handleMomentMatching();
void handlePrune();
void handleCap();
void handleNearestNeighbor();
void handlePDA();
void handleGaussianSum();
void handleGNN();

int main(int argc, char *argv[]) {

    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QGuiApplication app(argc, argv);

//    QQmlApplicationEngine engine;
//    const QUrl url(QStringLiteral("qrc:/ui/main.qml"));

//    DebugServer server(engine);
//    server.start();

//    UIConnectionHandler connectionHandler;
//    engine.rootContext()->setContextProperty("backend", &connectionHandler);
//    engine.load(url);

    // handleEllipsoidalGating();
    // handlePredictedLikelihood();
    // handleMomentMatching();
    // handlePrune();
    // handleCap();
    // handleNearestNeighbor();
    // handlePDA();
    // handleGaussianSum();
    handleGNN();

    return app.exec();
}

void getDataFromFile(QString path, QMap<QString, QString>& values) {

    QFile file(path);
    if(!file.open(QIODevice::ReadOnly)) {
        qDebug() << file.errorString();
    }

    QTextStream input(&file);

    while(!input.atEnd()) {
        QString line = input.readLine();
        QStringList fields = line.split(":");
        values[fields[1]] = fields[2];
    }

    file.close();
}

shared_ptr<VectorXd> getVectorXdData(const QMap<QString, QString>& values, QString field_name, const int LEN) {

    QStringList lst = values[field_name].split(",");
    shared_ptr<VectorXd> res = make_shared<VectorXd>(LEN);
    for(int i = 0; i < LEN; i++)
        (*res)(i, 0) = lst[i].toDouble();

    return res;
}

shared_ptr<Vector2d> getVector2dData(const QMap<QString, QString>& values, QString field_name, const int LEN) {

    shared_ptr<VectorXd> temp = getVectorXdData(values, field_name, LEN);
    shared_ptr<Vector2d> res = make_shared<Vector2d>( (*temp)(0, 0), (*temp)(1, 0) );
    return res;
}

shared_ptr<MatrixXd> getSquareMatrixXdData(const QMap<QString, QString>& values, QString field_name, const int LEN) {

    QStringList lst = values[field_name].split(",");
    shared_ptr<MatrixXd> res = make_shared<MatrixXd>(LEN, LEN);
    for(int i = 0; i < LEN; i++)
        for(int j = 0; j < LEN; j++)
            (*res)(i, j) = lst[(i * LEN) + j].toDouble();

    return res;
}

MatrixXd getMeasurementData(const QMap<QString, QString>& values) {

    QStringList lst_meas = values["var_meas"].split(",");
    int sz_measurements = (lst_meas.size() - 1) / 2;
    MatrixXd z(2, sz_measurements);
    for(int i = 0; i < sz_measurements; i++) {
        z(0, i) = lst_meas[i].toDouble();
        z(1, i) = lst_meas[i + sz_measurements].toDouble();
    }

    return z;
}

void handleEllipsoidalGating() {

    QMap<QString, QString> init_values;
    getDataFromFile("debug_data/SOT/2_1/init.txt", init_values);

    shared_ptr<TransitionConstantVelocity> transition_model =
             make_shared<TransitionConstantVelocity>(init_values["T"].toInt(), init_values["sigma_q"].toFloat());

    shared_ptr<MatrixXd> range_c = getSquareMatrixXdData(init_values, "range_c", 2);
    Sensor sensor(init_values["P_D"].toDouble(), init_values["lambda_c"].toDouble(), *range_c);

    shared_ptr<MeasurementConstantVelocity> measurement_model =
            make_shared<MeasurementConstantVelocity>(init_values["sigma_r"].toFloat());

    for(int i = 1; i <= 9; i++) {

        qDebug() << i << "-------------------------------";

        QMap<QString, QString> data_values;
        getDataFromFile(QString("debug_data/SOT/2_1/%1.txt").arg(i), data_values);

        shared_ptr<VectorXd> x = getVectorXdData(data_values, "var_x", 4);
        shared_ptr<MatrixXd> P = getSquareMatrixXdData(data_values, "var_P", 4);

        State state(x, P);

        Estimator estimator(measurement_model, transition_model);

        MatrixXd z = getMeasurementData(data_values);

        auto result = estimator.ellipsoidalGating(state, z, init_values["gating_size"].toFloat());
        shared_ptr<ArrayXi> meas_in_gate = std::get<0>(result);
        shared_ptr<MatrixXd> z_ingate = std::get<1>(result);

        qDebug() << data_values["indices"];
        for(int i = 0; i < meas_in_gate->size(); i++)
            // qDebug() << (*z_ingate)(0, i) << " " << (*z_ingate)(1, i);
            qDebug() << (*meas_in_gate)(i);
    }
}

void handlePredictedLikelihood() {

    QMap<QString, QString> init_values;
    getDataFromFile("debug_data/SOT/2_2/init.txt", init_values);

    shared_ptr<TransitionConstantVelocity> transition_model =
             make_shared<TransitionConstantVelocity>(init_values["T"].toInt(), init_values["sigma_q"].toFloat());

    shared_ptr<MatrixXd> range_c = getSquareMatrixXdData(init_values, "range_c", 2);
    Sensor sensor(init_values["P_D"].toDouble(), init_values["lambda_c"].toDouble(), *range_c);

    shared_ptr<MeasurementConstantVelocity> measurement_model =
            make_shared<MeasurementConstantVelocity>(init_values["sigma_r"].toFloat());

    for(int i = 1; i <= 9; i++) {

        qDebug() << i << "-------------------------------";

        QMap<QString, QString> data_values;
        getDataFromFile(QString("debug_data/SOT/2_2/%1.txt").arg(i), data_values);

        shared_ptr<VectorXd> x = getVectorXdData(data_values, "var_x", 4);
        shared_ptr<MatrixXd> P = getSquareMatrixXdData(data_values, "var_P", 4);


        State state(x, P);

        Estimator estimator(measurement_model, transition_model);

        MatrixXd z = getMeasurementData(data_values);

        shared_ptr<VectorXd> predicted_likelihood = estimator.predictedLikelihood(state, z);

        qDebug() << data_values["predict_likelihood"];
        for(int i = 0; i < predicted_likelihood->size(); i++)
            qDebug() << predicted_likelihood->coeff(i, 0);
    }
}

void handleMomentMatching() {

    QMap<QString, QString> init_values;
    getDataFromFile("debug_data/SOT/2_3/init.txt", init_values);

    int num_gaussians = init_values["numGaussians"].toInt();
    int num_dims = init_values["nDim"].toInt();

    QMap<QString, QString> res_values;
    getDataFromFile("debug_data/SOT/2_3/res.txt", res_values);

    shared_ptr<VectorXd> w = getVectorXdData(res_values, "w", num_gaussians);

    vector<State> states;
    for(int i = 1; i <= num_gaussians; i++) {

        QMap<QString, QString> data_values;
        getDataFromFile(QString("debug_data/SOT/2_3/%1.txt").arg(i), data_values);

        shared_ptr<VectorXd> x = getVectorXdData(data_values, "var_x", 4);
        shared_ptr<MatrixXd> P = getSquareMatrixXdData(data_values, "var_P", 4);

        states.push_back(State(x, P));
    }

    Estimator estimator;
    State state = estimator.momentMatching(*w, make_shared<vector<State>>(states));

    std::cout << state.getX() << std::endl << std::endl;
    std::cout << state.getP() << std::endl << std::endl;
    std::cout << res_values["var_x"].toStdString() << std::endl << std::endl;
    std::cout << res_values["var_P"].toStdString() << std::endl << std::endl;
}

void handlePrune() {

    QMap<QString, QString> values;
    getDataFromFile("debug_data/SOT/3_1/values.txt", values);

    float threshold = values["threshold"].toFloat();
    vector<int> states(100, 0);

    shared_ptr<VectorXd> weights = getVectorXdData(values, "hypothesesWeight", 100);

    Hypothesis hypothesis;
    shared_ptr<VectorXd> res_weights;
    shared_ptr<vector<int>> res_states;
    tie(res_weights, res_states) = hypothesis.prune(states, *weights, threshold);

    std::sort(std::begin(*res_weights), std::end(*res_weights));

    std::cout << values["hypothesesWeight_hat"].toStdString() << std::endl << std::endl;
    std::cout << res_weights->rows() << std::endl << std::endl;
    std::cout << (*res_weights) << std::endl;

}

void handleCap() {

    QMap<QString, QString> values;
    getDataFromFile("debug_data/SOT/3_2/values.txt", values);

    float threshold = values["threshold"].toFloat();
    vector<int> states(100, 0);

    shared_ptr<VectorXd> weights = getVectorXdData(values, "hypothesesWeight", 100);

    Hypothesis hypothesis;
    shared_ptr<VectorXd> res_weights;
    shared_ptr<vector<int>> res_states;
    tie(res_weights, res_states) = hypothesis.cap(states, *weights, threshold);

    std::sort(std::begin(*res_weights), std::end(*res_weights));

    std::cout << values["hypothesesWeight_hat"].toStdString() << std::endl << std::endl;
    std::cout << res_weights->rows() << std::endl << std::endl;
    std::cout << (*res_weights) << std::endl;
}

void handleNearestNeighbor() {

    QMap<QString, QString> init_values;
    getDataFromFile("debug_data/SOT/4_1/init.txt", init_values);

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

    shared_ptr<VectorXd> init_x = getVectorXdData(init_values, "initial_state_x", 5);
    shared_ptr<MatrixXd> init_P = getSquareMatrixXdData(init_values, "initial_state_P", 5);

    shared_ptr<Vector2d> s = getVector2dData(init_values, "s", 2);
    shared_ptr<MatrixXd> range_c = getSquareMatrixXdData(init_values, "range_c", 2);

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
        getDataFromFile(QString("debug_data/SOT/4_1/%1.txt").arg(i), data_values);

        std::cout << i << std::endl;
        MatrixXd z = getMeasurementData(data_values);
        tracker.step(z);
        // std::cout << tracker.getX() << std::endl << std::endl;
    }
}

void handlePDA() {

    QMap<QString, QString> init_values;
    getDataFromFile("debug_data/SOT/4_2/init.txt", init_values);

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

    shared_ptr<VectorXd> init_x = getVectorXdData(init_values, "initial_state_x", 5);
    shared_ptr<MatrixXd> init_P = getSquareMatrixXdData(init_values, "initial_state_P", 5);

    shared_ptr<Vector2d> s = getVector2dData(init_values, "s", 2);
    shared_ptr<MatrixXd> range_c = getSquareMatrixXdData(init_values, "range_c", 2);

    shared_ptr<Sensor> sensor = make_shared<Sensor>(P_D, lambda_c, *range_c);
    shared_ptr<Transition2dTurn> transition_model = make_shared<Transition2dTurn>(T, sigma_v, sigma_omega);
    shared_ptr<MeasurementRangeBearing> measurement_model = make_shared<MeasurementRangeBearing>(sigma_r, sigma_b, s);

    shared_ptr<State> init_state = make_shared<State>(init_x, init_P);

    shared_ptr<Estimator> estimator = make_shared<Estimator>(measurement_model, transition_model);
    TrackerPDA tracker(estimator, init_state, sensor, 13.8155, w_min);

    for(int i = 1; i <= 50; i++) {

        QMap<QString, QString> data_values;
        getDataFromFile(QString("debug_data/SOT/4_2/%1.txt").arg(i), data_values);

        std::cout << i << std::endl;
        MatrixXd z = getMeasurementData(data_values);
        tracker.step(z);
    }
}

void handleGaussianSum() {

    QMap<QString, QString> init_values;
    getDataFromFile("debug_data/SOT/4_3/init.txt", init_values);

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

    shared_ptr<VectorXd> init_x = getVectorXdData(init_values, "initial_state_x", 5);
    shared_ptr<MatrixXd> init_P = getSquareMatrixXdData(init_values, "initial_state_P", 5);

    shared_ptr<Vector2d> s = getVector2dData(init_values, "s", 2);
    shared_ptr<MatrixXd> range_c = getSquareMatrixXdData(init_values, "range_c", 2);

    shared_ptr<Sensor> sensor = make_shared<Sensor>(P_D, lambda_c, *range_c);
    shared_ptr<Transition2dTurn> transition_model = make_shared<Transition2dTurn>(T, sigma_v, sigma_omega);
    shared_ptr<MeasurementRangeBearing> measurement_model = make_shared<MeasurementRangeBearing>(sigma_r, sigma_b, s);

    shared_ptr<State> init_state = make_shared<State>(init_x, init_P);

    shared_ptr<Estimator> estimator = make_shared<Estimator>(measurement_model, transition_model);
    // TrackerNN tracker(estimator, init_state, sensor, 13.8155, w_min);
    TrackerGaussianSum tracker(estimator, init_state, sensor, 13.8155, M, w_min);

    for(int i = 1; i <= 50; i++) {

        QMap<QString, QString> data_values;
        getDataFromFile(QString("debug_data/SOT/4_3/%1.txt").arg(i), data_values);

        std::cout << "i: " << i << std::endl;
        MatrixXd z = getMeasurementData(data_values);
        tracker.step(z);
    }
}


void handleGNN() {

    QMap<QString, QString> init_values;
    getDataFromFile("debug_data/MOT/1_1/init.txt", init_values);

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

    shared_ptr<Vector2d> s = getVector2dData(init_values, "s", 2);
    shared_ptr<MatrixXd> range_c = getSquareMatrixXdData(init_values, "range_c", 2);

    shared_ptr<Sensor> sensor = make_shared<Sensor>(P_D, lambda_c, *range_c);
    shared_ptr<Transition2dTurn> transition_model = make_shared<Transition2dTurn>(T, sigma_v, sigma_omega);
    shared_ptr<MeasurementRangeBearing> measurement_model = make_shared<MeasurementRangeBearing>(sigma_r, sigma_b, s);

    PtrVecState states = make_shared<vector<shared_ptr<State>>>();

    for(int i = 1; i <= 4; i++) {

        QMap<QString, QString> state_values;
        getDataFromFile(QString("debug_data/MOT/1_1/init_states_%1.txt").arg(i), state_values);

        shared_ptr<VectorXd> x = getVectorXdData(state_values, "var_x", 5);
        shared_ptr<MatrixXd> P = getSquareMatrixXdData(state_values, "var_P", 5);

        states->push_back(make_shared<State>(x, P));
    }

    shared_ptr<Estimator> estimator = make_shared<Estimator>(measurement_model, transition_model);

    MultiTrackerGNN tracker(estimator, states, sensor, 13.8155, M, w_min);

    for(int i = 1; i <= 20; i++) {

        QMap<QString, QString> data_values;
        getDataFromFile(QString("debug_data/MOT/1_1/%1.txt").arg(i), data_values);

        std::cout << "i: " << i << "--------------------------" << std::endl;
        MatrixXd z = getMeasurementData(data_values);
        tracker.step(z);
    }
}










