
#include <QCoreApplication>
#include "inc/debug_server.h"


int main(int argc, char *argv[]) {

    QCoreApplication a(argc, argv);

    DebugServer server;
    server.start();

    return a.exec();
}
