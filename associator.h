#ifndef ASSOCIATOR_H
#define ASSOCIATOR_H

#include <QObject>
#include <QList>
#include "state.h"
#include "detection.h"
#include "estimator.h"
#include "multihypothesis.h"

class Associator : public QObject
{
    Q_OBJECT
public:
    explicit Associator(Estimator *estimator, float spatialClutterIntensity = 0.125, float P_D = 0.85, float P_G = 0.95,
                        QObject *parent = nullptr);

    virtual QList<MultiHypothesis *> * associate(const QList<State *> &objects, const QList<Detection *> &detections, int dt)
                { return nullptr;  }
    virtual MultiHypothesis * hypothesis(State *prior, const QList<Detection *> &detections, int dt)
            { std::ignore = prior; std::ignore = detections; std::ignore = dt;
              return nullptr;  }

protected:
    Estimator *estimator;
    float P_D;
    float P_G;
    float spatialClutterIntensity;

signals:

};

#endif // ASSOCIATOR_H
