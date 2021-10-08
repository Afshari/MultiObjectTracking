
#include <QCoreApplication>
#include <iostream>

#include "tcpserver.h"

int main(int argc, char *argv[]) {

    QCoreApplication a(argc, argv);

    TCPServer tcpServer;
    tcpServer.start();


    return a.exec();
}
