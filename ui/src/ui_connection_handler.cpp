#include "ui/inc/ui_connection_handler.h"

UIConnectionHandler::UIConnectionHandler(QObject *parent) : QObject(parent) {

    counter = 0;

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    // timer->start(1000);
}


void UIConnectionHandler::update() {

    counter += 1;
    qDebug() << "Timer is Running ...";
    emit this->recvData(QString::number(counter));
    emit this->hello();
}

void UIConnectionHandler::receiveFromQml(QString value) {

    qDebug() << "Received: " << value;
}

