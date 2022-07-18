
#include <iostream>
#include <QCoreApplication>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickView>
#include <QQmlContext>
#include "ui/inc/ui_handler.h"

int main(int argc, char *argv[]) {

    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;
    const QUrl url(QStringLiteral("qrc:/ui/main.qml"));

    UIHandler uiHandler(engine);
    //engine.rootContext()->setContextProperty("backend", &uiHandler);
    engine.load(url);

    return app.exec();


}
