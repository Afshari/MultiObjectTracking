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
            recvMeasurements = new QList<Detection *>();
            for(int i = 1; i < receivedArr.size(); i+=3) {
                if(receivedArr[i].toUInt() == 23) {

                    QStringList dim = receivedArr[i+1].split(',');
                    QStringList items = receivedArr[i+2].split(',');

                    Detection *detection = new Detection(Detection::DetectionType::detect);
                    VectorXd *measurement = new VectorXd(dim[0].toUInt());
                    for(int j = 0; j < (*measurement).size(); j++) {
                        (*measurement)[j] = items[j].toFloat();
                    }
                    detection->x = measurement;
                    recvMeasurements->append(detection);
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
            MeasurementPrediction measurementPrediction(&recvStateMeasurement, nullptr, nullptr, nullptr, recvUpsilon);
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

            State *predicted = kalman->predict(prior, dt);
            std::cout << predicted->getX() << std::endl;
            std::cout << "---------------" << std::endl;
            std::cout << predicted->getP() << std::endl;
            std::cout << "---------------" << std::endl;
            std::cout << "---------------" << std::endl;
            std::cout << "---------------" << std::endl;

        } else if(code == 82) {

            TransitionLinearGaussian *transitionLinearGaussian = new TransitionLinearGaussian(0.005);

            KalmanFilter *kalman = new KalmanFilter(measurementModel, transitionLinearGaussian);

            StateGaussian prior(recvX, recvP);

            State *predicted = kalman->predict(prior, dt);
            MeasurementPrediction *meas = kalman->predictMeasurement(predicted);

            std::cout << "---Predicted x------------" << std::endl;
            std::cout << predicted->getX() << std::endl;
            std::cout << "---Predicted Measurement x------------" << std::endl;
            std::cout << *meas->xOfZPred << std::endl;
//            std::cout << "---Predicted P------------" << std::endl;
//            std::cout << predicted->getP() << std::endl;
            std::cout << "---Measurement x------------" << std::endl;
            std::cout << meas->statePred->getX() << std::endl;
//            std::cout << "---Measurement P------------" << std::endl;
//            std::cout << meas->state->getP() << std::endl;
            std::cout << "---------------" << std::endl;
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
            State *predicted = kalman->predict(prior, dt);
            MeasurementPrediction *meas = kalman->predictMeasurement(predicted);

//            std::cout << recvMeasurements->length() << std::endl;
            PDA pda(kalman);
            for(int i = 0; i < recvMeasurements->length(); i++) {
//                std::cout << "recvMeasurements " << *(*recvMeasurements)[i] << std::endl;
//                std::cout << "meas->state->getX() " << *meas->zPred << std::endl;
//                std::cout << "meas->S)" << *meas->S << std::endl;
                std::cout << "log pdf: " << pda.logPDF(*(*recvMeasurements)[i]->x, *meas->zPred, *meas->S) << std::endl;
//                std::cout << *(*recvMeasurements)[i] << std::endl;
            }
            std::cout << "--------------" << std::endl;

        } else if(code == 84) {

            TransitionLinearGaussian *transitionLinearGaussian = new TransitionLinearGaussian(0.005);
            KalmanFilter *kalman = new KalmanFilter(measurementModel, transitionLinearGaussian);
            StateGaussian prior(recvX, recvP);
            State *predicted = kalman->predict(prior, dt);
            MeasurementPrediction *meas = kalman->predictMeasurement(predicted);

            PDA pda(kalman, 0.125, 0.9);

            QList<SingleHypothesis *> list;
            Detection *missDetection = new Detection(Detection::DetectionType::miss);
            SingleHypothesis *single = new SingleHypothesis(missDetection, predicted, transitionLinearGaussian,
                                                           nullptr, 1 - pda.P_D*pda.P_G);
            list.append(single);

//            std::cout << "x Pred: \r\n" << meas->statePred->getX() << std::endl;
//            std::cout << "predic: \r\n" << predicted->getX() << std::endl;

            for(int i = 0; i < recvMeasurements->length(); i++) {

                double log_pdf = pda.logPDF(*(*recvMeasurements)[i]->x, *meas->zPred, *meas->S);
                double probability = pda.getProbability(log_pdf);
//                std::cout << probability << std::endl;
                SingleHypothesis *single = new SingleHypothesis((*recvMeasurements)[i], nullptr, transitionLinearGaussian,
                                                               meas, probability);
                list.append(single);
//                std::cout << *(*recvMeasurements)[i] << std::endl;
            }

            MultiHypothesis multi(&list);
            multi.normalizeWeights();

            QString res = "";

            for(int i = 0; i < multi.items->length(); i++) {
//               std::cout << "normal probability: " << multi.items->at(i)->probability << std::endl;
                if(res != "")
                    res += ", ";
                res += QString::number(multi.items->at(i)->probability);
            }

            socket->write( res.toStdString().c_str(), res.length() );

//            std::cout << "Result: " << res.toStdString() << std::endl;
//            std::cout << std::endl;

        } else if(code == 85) {

            TransitionLinearGaussian *transitionLinearGaussian = new TransitionLinearGaussian(0.005);
            KalmanFilter *kalman = new KalmanFilter(measurementModel, transitionLinearGaussian);
            StateGaussian prior(recvX, recvP);

            PDA pda(kalman, 0.125, 0.9);

            QList<State *> objects;
            objects.append(&prior);

            QList<MultiHypothesis *> *res = pda.associate(objects, *recvMeasurements, dt);


            for(int i = 0; i < res->length(); i++) {
                QList<SingleHypothesis *> *currItems = res->at(i)->items;

                QString dataToSend = "";
                for(int j = 0; j < currItems->length(); j++) {
                    QString currData = "";

//                    std::cout << currItems->at(j)->probability << std::endl;
//                    std::cout << currItems->at(j)->state->getX() << std::endl;
//                    std::cout << currItems->at(j)->state->getP() << std::endl;
//                    std::cout << "S: \r\n" << currItems->at(j)->measurementPrediction->S << std::endl;
//                    std::cout << "Upsilon: \r\n" << currItems->at(j)->measurementPrediction->upsilon << std::endl;

                    SingleHypothesis *currHypothesis = currItems->at(j);

//                    std::cout << currItems->at(j)->probability << std::endl;
                    if(currHypothesis->detection->type == Detection::DetectionType::detect) {
//                        std::cout << "Got Data: \r\n" << *currHypothesis->measurementPrediction->S << std::endl;
//                        std::cout << "--------------" << std::endl;

//                        std::cout << "zPred: \r\n" << *currHypothesis->measurementPrediction->zPred << std::endl;

                        currData += "1 | ";
                        currData += QString("%1 | %2 | %3 | %4 | %5 | %6 | %7").arg(
                                    QString::fromStdString(vectorToStr(currHypothesis->state->getX())),
                                    QString::fromStdString(matrixToStr(currHypothesis->state->getP())),
                                    QString::fromStdString(vectorToStr(*currHypothesis->detection->x)),
                                    QString::fromStdString(matrixToStr(*currHypothesis->measurementPrediction->S)),
                                    QString::fromStdString(matrixToStr(*currHypothesis->measurementPrediction->upsilon)),
                                    QString::fromStdString(vectorToStr(*currHypothesis->measurementPrediction->zPred)),
                                    QString::number(currHypothesis->probability)
                                    );
//                                    QString::fromStdString(matrixToStr(*currHypothesis->measurementPrediction->S))
//                                    );
//                        currHypothesis->state->getX();

                    } else if(currHypothesis->detection->type == Detection::DetectionType::miss) {
                        currData += "0 | ";
                    }

                    if(dataToSend != "")
                        dataToSend += ", ";
                    dataToSend += currData;

//                    res += QString("%1 | %2").arg(
//                               QString::number(currItems->at(j)->probability),
//                               QString::fromStdString(matrixToStr(*currHypothesis->measurementPrediction->S)));

//                    res += QString::number(currItems->at(j)->probability);
                }
                socket->write( dataToSend.toStdString().c_str(), dataToSend.length() );
            }


        }

    }

}











