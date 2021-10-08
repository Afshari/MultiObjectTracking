#ifndef DEBUG_DENSITY_H
#define DEBUG_DENSITY_H

#include <QObject>


class DebugDensity : public QObject
{
    Q_OBJECT
public:
    explicit DebugDensity(QObject *parent = nullptr);

signals:

};

#endif // DEBUG_DENSITY_H
