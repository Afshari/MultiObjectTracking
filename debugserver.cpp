#include "debugserver.h"

DebugServer::DebugServer(QObject *parent) : QObject(parent) {

    MatrixXd mNoiseCovar = 0.75 * MatrixXd::Identity(2, 2);
    measurementModel = new MeasurementLinearGaussian(&mNoiseCovar);


    connect(&server, &QTcpServer::newConnection, this, &DebugServer::newConnection);
}

void DebugServer::start() {

    server.listen(QHostAddress::Any, 6060);
}

void DebugServer::quit() {

    server.close();
}

void DebugServer::newConnection() {

    QTcpSocket *socket = server.nextPendingConnection();
    connect(socket, &QTcpSocket::disconnected, this, &DebugServer::disconnected);
    connect(socket, &QTcpSocket::readyRead, this, &DebugServer::readyRead);

    qInfo() << "connected: " << socket;
}

void DebugServer::disconnected() {

    QTcpSocket *socket = qobject_cast<QTcpSocket *>(sender());
    qInfo() << "Disconnected" << socket;
    qInfo() << "Parent" << socket->parent();

    socket->deleteLater();
}

std::string vectorToStr(const Eigen::VectorXd& vec){
    std::stringstream ss;
    ss << vec;
    return ss.str();
}

void DebugServer::readyRead() {

    QTcpSocket *socket = qobject_cast<QTcpSocket *>(sender());
//    qInfo() << "Ready Read" << socket;
    QString received = QString::fromUtf8( socket->readAll() );

    QStringList splitData = received.split('|');

    for(int k = 0; k < splitData.size(); k++) {

        QStringList receivedArr = splitData[k].split(':');

        if(receivedArr[0].toUInt() == 10) {
    //        qInfo() << "Measurement State Vector";
    //        qInfo() << receivedArr[1];
    //        qInfo() << receivedArr[2];

            QStringList dim = receivedArr[1].split(',');
            QStringList items = receivedArr[2].split(',');

            VectorXd measurement(dim[0].toUInt());
            for(int i = 0; i < measurement.size(); i++) {
                measurement[i] = items[i].toFloat();
            }

            recvMeasurement = new VectorXd(measurement);

    //        std::cout << "Measurement State Vector: \r\n" << measurement << std::endl;
//            socket->write("OK\r\n", 4);

        } else if(receivedArr[0].toUInt() == 11) {
    //        qInfo() << "Prediction State Vector";
    //        qInfo() << receivedArr[1];
    //        qInfo() << receivedArr[2];

            QStringList dim = receivedArr[1].split(',');
            QStringList items = receivedArr[2].split(',');

            VectorXd predState(dim[0].toUInt());
            for(int i = 0; i < predState.size(); i++) {
                predState[i] = items[i].toFloat();
            }

            recvX = new VectorXd(predState);

    //        std::cout << "Prediction State Vector: \r\n" << predState << std::endl;
//            socket->write("OK\r\n", 4);

        } else if(receivedArr[0].toUInt() == 12) {
    //        qInfo() << "Prediction Covar";
    //        qInfo() << receivedArr[1];
    //        qInfo() << receivedArr[2];

            QStringList dim = receivedArr[1].split(',');
            QStringList items = receivedArr[2].split(',');

            MatrixXd predCov(dim[0].toUInt(), dim[1].toUInt());
            for(int i = 0; i < predCov.rows(); i++) {
                for(int j = 0; j < predCov.cols(); j++) {
                    predCov(i, j) = items[ (i*dim[1].toUInt())  + j].toFloat();
                }
            }

            recvP = new MatrixXd(predCov);

    //        std::cout << "Prediction Cov: \r\n" << predCov << std::endl;
//            socket->write("OK\r\n", 4);

        } else if(receivedArr[0].toUInt() == 13) {
    //        qInfo() << "Measurement Prediction State Vector";
    //        qInfo() << receivedArr[1];
    //        qInfo() << receivedArr[2];

            QStringList dim = receivedArr[1].split(',');
            QStringList items = receivedArr[2].split(',');

            VectorXd measurementPredState(dim[0].toUInt());
            for(int i = 0; i < measurementPredState.size(); i++) {
                measurementPredState[i] = items[i].toFloat();
            }

            recvXPredMeas = new VectorXd(measurementPredState);

    //        std::cout << "Measurement Prediction State Vector: \r\n" << measurementPredState << std::endl;
//            socket->write("OK\r\n", 4);

        } else if(receivedArr[0].toUInt() == 14) {
    //        qInfo() << "Measurement Prediction Covar";
    //        qInfo() << receivedArr[1];
    //        qInfo() << receivedArr[2];

            QStringList dim = receivedArr[1].split(',');
            QStringList items = receivedArr[2].split(',');

            MatrixXd measurementPredCov(dim[0].toUInt(), dim[1].toUInt());
            for(int i = 0; i < measurementPredCov.rows(); i++) {
                for(int j = 0; j < measurementPredCov.cols(); j++) {
                    measurementPredCov(i, j) = items[ (i*dim[1].toUInt())  + j].toFloat();
                }
            }

            recvPPredMeas = new MatrixXd(measurementPredCov);

    //        std::cout << "Measurement Prediction Covar: \r\n" << measurementPredCov << std::endl;
//            socket->write("OK\r\n", 4);

        } else if(receivedArr[0].toUInt() == 15) {
    //        qInfo() << "Measurement Prediction CrossCovar";
    //        qInfo() << receivedArr[1];
    //        qInfo() << receivedArr[2];

            QStringList dim = receivedArr[1].split(',');
            QStringList items = receivedArr[2].split(',');

            MatrixXd measurementPredCrossCovar(dim[0].toUInt(), dim[1].toUInt());
            for(int i = 0; i < measurementPredCrossCovar.rows(); i++) {
                for(int j = 0; j < measurementPredCrossCovar.cols(); j++) {
                    measurementPredCrossCovar(i, j) = items[ (i*dim[1].toUInt())  + j].toFloat();
                }
            }

            recvCrossCov = new MatrixXd(measurementPredCrossCovar);

    //        std::cout << "Measurement Prediction CrossCovar: \r\n" << measurementPredCrossCovar << std::endl;
//            socket->write("OK\r\n", 4);

        } else if(receivedArr[0].toInt() == 80) {


            StateGaussian state(recvX, recvP);

            StateGaussian recvStateMeasurement(recvXPredMeas, recvPPredMeas);

            MeasurementPrediction measurementPrediction(&recvStateMeasurement, nullptr, nullptr, recvCrossCov);

            TransitionLinearGaussian *transitionLinearGaussian = new TransitionLinearGaussian(0.005);

            KalmanFilter *kalman = new KalmanFilter(measurementModel, transitionLinearGaussian);
            State* posteriorState = kalman->update(state, *measurementModel, *recvMeasurement, measurementPrediction);

            std::cout << "Posterior" << std::endl;
            std::cout << posteriorState->getX() << std::endl;
            std::cout << posteriorState->getP() << std::endl;
            std::cout << "--------------" << std::endl;

//            socket->write("OK\r\n", 4);
            socket->write(vectorToStr(posteriorState->getX()).c_str(), vectorToStr(posteriorState->getX()).length() );
        }

    }

//    socket->write("OK\r\n", 4);
}











