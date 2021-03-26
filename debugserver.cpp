#include "debugserver.h"

DebugServer::DebugServer(QObject *parent) : QObject(parent) {

    MatrixXd *mNoiseCovar = new MatrixXd(.75 * MatrixXd::Identity(2, 2));
    measurementModel = new MeasurementLinearGaussian(mNoiseCovar);


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

std::string matrixToStr(const Eigen::MatrixXd& mat){
    std::stringstream ss;
    ss << mat;
    return ss.str();
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

//    qInfo() << received;

    QStringList splitData = received.split('|');

    for(int k = 0; k < splitData.size(); k++) {

        QStringList receivedArr = splitData[k].split(':');
        int code = receivedArr[0].toUInt();

//        std::cout << code << std::endl;

        if(code == 10) {

            QStringList dim = receivedArr[1].split(',');
            QStringList items = receivedArr[2].split(',');

            VectorXd measurement(dim[0].toUInt());
            for(int i = 0; i < measurement.size(); i++) {
                measurement[i] = items[i].toFloat();
            }

            recvMeasurement = new VectorXd(measurement);

        } else if(code == 11) {

            QStringList dim = receivedArr[1].split(',');
            QStringList items = receivedArr[2].split(',');

            VectorXd predState(dim[0].toUInt());
            for(int i = 0; i < predState.size(); i++) {
                predState[i] = items[i].toFloat();
            }

            recvX = new VectorXd(predState);

        } else if(code == 12) {

            QStringList dim = receivedArr[1].split(',');
            QStringList items = receivedArr[2].split(',');

            MatrixXd predCov(dim[0].toUInt(), dim[1].toUInt());
            for(int i = 0; i < predCov.rows(); i++) {
                for(int j = 0; j < predCov.cols(); j++) {
                    predCov(i, j) = items[ (i*dim[1].toUInt())  + j].toFloat();
                }
            }

            recvP = new MatrixXd(predCov);

        } else if(code == 13) {

            QStringList dim = receivedArr[1].split(',');
            QStringList items = receivedArr[2].split(',');

            VectorXd measurementPredState(dim[0].toUInt());
            for(int i = 0; i < measurementPredState.size(); i++) {
                measurementPredState[i] = items[i].toFloat();
            }

            recvXPredMeas = new VectorXd(measurementPredState);

        } else if(code == 14) {

            QStringList dim = receivedArr[1].split(',');
            QStringList items = receivedArr[2].split(',');

            MatrixXd measurementPredCov(dim[0].toUInt(), dim[1].toUInt());
            for(int i = 0; i < measurementPredCov.rows(); i++) {
                for(int j = 0; j < measurementPredCov.cols(); j++) {
                    measurementPredCov(i, j) = items[ (i*dim[1].toUInt())  + j].toFloat();
                }
            }

            recvPPredMeas = new MatrixXd(measurementPredCov);

        } else if(code == 15) {

            QStringList dim = receivedArr[1].split(',');
            QStringList items = receivedArr[2].split(',');

            MatrixXd upsilon(dim[0].toUInt(), dim[1].toUInt());
            for(int i = 0; i < upsilon.rows(); i++) {
                for(int j = 0; j < upsilon.cols(); j++) {
                    upsilon(i, j) = items[ (i*dim[1].toUInt())  + j].toFloat();
                }
            }

            recvUpsilon = new MatrixXd(upsilon);

        } else if(code == 20) {

            QStringList dim = receivedArr[1].split(',');
            QStringList items = receivedArr[2].split(',');

            VectorXd predState(dim[0].toUInt());
            for(int i = 0; i < predState.size(); i++) {
                predState[i] = items[i].toFloat();
            }

            recvX = new VectorXd(predState);
//            std::cout << "x: " << *recvX << std::endl;

        } else if(code == 21) {

            QStringList dim = receivedArr[1].split(',');
            QStringList items = receivedArr[2].split(',');

            MatrixXd predCov(dim[0].toUInt(), dim[1].toUInt());
            for(int i = 0; i < predCov.rows(); i++) {
                for(int j = 0; j < predCov.cols(); j++) {
                    predCov(i, j) = items[ (i*dim[1].toUInt())  + j].toFloat();
                }
            }

            recvP = new MatrixXd(predCov);
//            std::cout << "P: " << *recvP << std::endl;

        } else if(code == 22) {

//            qInfo() << receivedArr;
            recvMeasurements = new QList<VectorXd *>();
            for(int i = 1; i < receivedArr.size(); i+=3) {
                if(receivedArr[i].toUInt() == 23) {

                    QStringList dim = receivedArr[i+1].split(',');
                    QStringList items = receivedArr[i+2].split(',');

                    VectorXd *measurement = new VectorXd(dim[0].toUInt());
                    for(int j = 0; j < (*measurement).size(); j++) {
                        (*measurement)[j] = items[j].toFloat();
                    }
                    recvMeasurements->append(measurement);
                }
            }

//            for(int i = 0; i < (*recvMeasurements).size(); i++) {
//                VectorXd *measurement = (*recvMeasurements)[i];
//                std::cout << *measurement << std::endl;
//                std::cout << "----" << std::endl;
//            }
        } else if(code == 24) {
            dt = receivedArr[1].toUInt();
        }

        else if(code == 80) {

            StateGaussian state(recvX, recvP);
            StateGaussian recvStateMeasurement(recvXPredMeas, recvPPredMeas);
            MeasurementPrediction measurementPrediction(&recvStateMeasurement, nullptr, nullptr, recvUpsilon);
            TransitionLinearGaussian *transitionLinearGaussian = new TransitionLinearGaussian(0.005);

            KalmanFilter *kalman = new KalmanFilter(measurementModel, transitionLinearGaussian);
            State* posteriorState = kalman->update(state, *measurementModel, *recvMeasurement, measurementPrediction);

            std::cout << "Posterior" << std::endl;
            std::cout << posteriorState->getX() << std::endl;
            std::cout << posteriorState->getP() << std::endl;
            std::cout << "--------------" << std::endl;

            QString response = QString("%1 | %2").arg(QString::fromStdString(vectorToStr(posteriorState->getX())),
                                                      QString::fromStdString(matrixToStr(posteriorState->getP())) );
            socket->write(  response.toStdString().c_str(),
                            response.length() );
        } else if(code == 81) {

            TransitionLinearGaussian *transitionLinearGaussian = new TransitionLinearGaussian(0.005);

            KalmanFilter *kalman = new KalmanFilter(measurementModel, transitionLinearGaussian);

            StateGaussian prior(recvX, recvP);

            State *posterior = kalman->predict(prior, dt);
            std::cout << posterior->getX() << std::endl;
            std::cout << "---------------" << std::endl;
            std::cout << posterior->getP() << std::endl;
            std::cout << "---------------" << std::endl;
            std::cout << "---------------" << std::endl;
            std::cout << "---------------" << std::endl;

        } else if(code == 82) {

            TransitionLinearGaussian *transitionLinearGaussian = new TransitionLinearGaussian(0.005);

            KalmanFilter *kalman = new KalmanFilter(measurementModel, transitionLinearGaussian);

            StateGaussian prior(recvX, recvP);

            State *posterior = kalman->predict(prior, dt);
            MeasurementPrediction *meas = kalman->predictMeasurement(posterior);

            std::cout << meas->state->getX() << std::endl;
//            std::cout << "---------------" << std::endl;
//            std::cout << meas->state->getP() << std::endl;
//            std::cout << "---------------" << std::endl;
//            std::cout << *meas->upsilon << std::endl;
//            std::cout << "---------------" << std::endl;
//            std::cout << *meas->S << std::endl;
//            std::cout << "---------------" << std::endl;
//            std::cout << "---------------" << std::endl;
//            std::cout << "---------------" << std::endl;

        } else if(code == 83) {

            TransitionLinearGaussian *transitionLinearGaussian = new TransitionLinearGaussian(0.005);
            KalmanFilter *kalman = new KalmanFilter(measurementModel, transitionLinearGaussian);
            StateGaussian prior(recvX, recvP);
            State *posterior = kalman->predict(prior, dt);
            MeasurementPrediction *meas = kalman->predictMeasurement(posterior);

//            std::cout << recvMeasurements->length() << std::endl;
            PDA pda;
            for(int i = 0; i < recvMeasurements->length(); i++) {
//                std::cout << "recvMeasurements " << *(*recvMeasurements)[i] << std::endl;
//                std::cout << "meas->state->getX() " << *meas->zPred << std::endl; ////////
//                std::cout << "meas->S)" << *meas->S << std::endl;
                std::cout << "log pdf: " << pda.logPDF(*(*recvMeasurements)[i], *meas->zPred, *meas->S) << std::endl;
//                std::cout << *(*recvMeasurements)[i] << std::endl;
            }
            std::cout << "--------------" << std::endl;

        } else if(code == 84) {

            TransitionLinearGaussian *transitionLinearGaussian = new TransitionLinearGaussian(0.005);
            KalmanFilter *kalman = new KalmanFilter(measurementModel, transitionLinearGaussian);
            StateGaussian prior(recvX, recvP);
            State *posterior = kalman->predict(prior, dt);
            MeasurementPrediction *meas = kalman->predictMeasurement(posterior);

            PDA pda(0.125, 0.9);

            QList<SingleHypothesis *> list;
            SingleHypothesis *singl = new SingleHypothesis(nullptr, nullptr, nullptr, nullptr, 1 - pda.P_D*pda.P_G);
            list.append(singl);

            for(int i = 0; i < recvMeasurements->length(); i++) {
                double log_pdf = pda.logPDF(*(*recvMeasurements)[i], *meas->zPred, *meas->S);
                double probability = pda.getProbability(log_pdf);
                std::cout << probability << std::endl;
                SingleHypothesis *singl = new SingleHypothesis(nullptr, nullptr, nullptr, nullptr, probability);
                list.append(singl);
//                std::cout << *(*recvMeasurements)[i] << std::endl;
            }
            MultiHypothesis multi(&list);
            multi.normalizeWeights();

            for(int i = 0; i < multi.items->length(); i++) {
               std::cout << "normal probability: " << multi.items->at(i)->probability << std::endl;
            }
            std::cout << std::endl;

        }

    }

}











