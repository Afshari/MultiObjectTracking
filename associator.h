#ifndef ASSOCIATOR_H
#define ASSOCIATOR_H

#include <QObject>
#include <QList>
#include "stategaussian.h"
#include "detection.h"

class Associator : public QObject
{
    Q_OBJECT
public:
    explicit Associator(float spatialClutterIntensity = 0.125, float P_D = 0.85, float P_G = 0.95,
                        QObject *parent = nullptr);

    virtual void associate()  {  }
    virtual void hypothesis(const State &state, const QList<Detection *> &detections)
            { std::ignore = state; std::ignore = detections; }

protected:
    float P_D;
    float P_G;
    float spatialClutterIntensity;

signals:

};

#endif // ASSOCIATOR_H
