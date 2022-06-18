#include "inc/tcpserver.h"

TCPServer::TCPServer(QObject *parent) : QObject(parent) {

//    MatrixXd *mNoiseCovar = new MatrixXd(0.75 * MatrixXd::Identity(2, 2));
//    measurementModel = new MeasurementLinearGaussian(mNoiseCovar);

//    transitionLinearGaussian = new TransitionLinearGaussian(0.005);
//    kalman = new KalmanFilter(measurementModel, transitionLinearGaussian);
////    prior = new StateGaussian(recvX, recvP);
//    pda = new PDA(kalman, 0.125, 0.9);

////    sendCounter = 0;

//    pdaTracker = new PDATracker();

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

    }
}























