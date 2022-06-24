
#include <QCoreApplication>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickView>
#include <QQmlContext>
#include "inc/debug_server.h"
#include "ui/inc/ui_connection_handler.h"
#include "inc/debug_run.h"


int main(int argc, char *argv[]) {

    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;
    const QUrl url(QStringLiteral("qrc:/ui/main.qml"));

//    DebugServer server(engine);
//    server.start();
    DebugRun debug_run(engine);
    debug_run.run();


//    UIConnectionHandler connectionHandler;
//    engine.rootContext()->setContextProperty("backend", &connectionHandler);
    engine.load(url);


    return app.exec();
}






