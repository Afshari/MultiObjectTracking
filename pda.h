#ifndef PDA_H
#define PDA_H

#include <qmath.h>
#include "associator.h"
#include "stats.hpp"
#include "estimator.h"
#include "singlehypothesis.h"
#include "multihypothesis.h"

class PDA : public Associator {

public:
    PDA(Estimator *estimator, float spatialClutterIntensity = 0.125, float P_D = 0.85, float P_G = 0.95);

    virtual QList<MultiHypothesis *> * associate(const QList<State *> &objects, const QList<Detection *> &detections, int dt) override;
    virtual MultiHypothesis * hypothesis(State *prior, const QList<Detection *> &detections, int dt) override;

protected:
    double logPDF(const VectorXd &xDetection, const VectorXd &xMeasPred, const MatrixXd &PMeasPred);
    double toPDF(const double &logPDF);
    double getProbability(const double &logPDF);

    friend class TestPDA;
    friend class DebugServer;

};

#endif // PDA_H
