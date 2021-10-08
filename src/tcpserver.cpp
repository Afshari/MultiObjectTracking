#include "inc/tcpserver.h"

TCPServer::TCPServer(QObject *parent) : QObject(parent) {

    MatrixXd *mNoiseCovar = new MatrixXd(0.75 * MatrixXd::Identity(2, 2));
    measurementModel = new MeasurementLinearGaussian(mNoiseCovar);

    transitionLinearGaussian = new TransitionLinearGaussian(0.005);
    kalman = new KalmanFilter(measurementModel, transitionLinearGaussian);
//    prior = new StateGaussian(recvX, recvP);
    pda = new PDA(kalman, 0.125, 0.9);

//    sendCounter = 0;

    pdaTracker = new PDATracker();

    connect(&server, &QTcpServer::newConnection, this, &TCPServer::newConnection);
}

void TCPServer::start() {

    server.listen(QHostAddress::Any, 6060);
}

void TCPServer::quit() {

    server.close();
}

void TCPServer::newConnection() {

    QTcpSocket *socket = server.nextPendingConnection();
    connect(socket, &QTcpSocket::disconnected, this, &TCPServer::disconnected);
    connect(socket, &QTcpSocket::readyRead, this, &TCPServer::readyRead);

    qInfo() << "connected: " << socket;
}

void TCPServer::disconnected() {

    QTcpSocket *socket = qobject_cast<QTcpSocket *>(sender());
    qInfo() << "Disconnected" << socket;
//    qInfo() << "Parent" << socket->parent();

    socket->deleteLater();
}


void TCPServer::readyRead() {

    QTcpSocket *socket = qobject_cast<QTcpSocket *>(sender());
    QString received = QString::fromUtf8( socket->readAll() );

    QStringList splitData = received.split('|');

    for(int k = 0; k < splitData.size(); k++) {

        QStringList receivedArr = splitData[k].split(':');
        int code = receivedArr[0].toUInt();



        if(code == 20) {

//            dt = 0;
//            std::cout << "prior x: \r\n" << predState << std::endl;
            pdaTracker->initialize();
            pdaTracker->interpretX(receivedArr);

        } else if(code == 21) {

//            std::cout << "prior cov: \r\n" << predCov << std::endl;

            pdaTracker->interpretP(receivedArr);

        } else if(code == 22) {

            pdaTracker->interpretMeasurements(receivedArr);

        } else if(code == 27) {

            pdaTracker->setScale(receivedArr[1].toUInt());

        } else if(code == 89) {

            QString response = pdaTracker->trackLoop();
            socket->write( response.toStdString().c_str(), response.length() );

//            std::cout << "**********" << std::endl;
//            prior = new StateGaussian(recvX, recvP);
//            dt = 1;

        }
    }
}























