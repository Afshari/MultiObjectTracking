#ifndef PDA_H
#define PDA_H

#include <qmath.h>
#include "associator.h"
#include "stats.hpp"


class PDA : public Associator {

public:
    PDA(float spatialClutterIntensity = 0.125, float P_D = 0.85, float P_G = 0.95);

    virtual void associate() override;
    virtual void hypothesis(const State &state, const QList<Detection *> &detections) override;

protected:
    double logPDF(const VectorXd &xDetection, const VectorXd &xMeasPred, const MatrixXd &PMeasPred);
    double toPDF(const double &logPDF);
    double getProbability(const double &logPDF);

    friend class TestPDA;
    friend class DebugServer;

};

#endif // PDA_H
