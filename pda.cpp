#include "pda.h"

PDA::PDA(float spatialClutterIntensity, float P_D , float P_G) : Associator(spatialClutterIntensity, P_D, P_G) {

}
void PDA::associate() {

}

void PDA::hypothesis(const State &state, const QList<Detection *> &detections) {

//    std::cout << detections[0].x << std::endl;
}

double PDA::logPDF(const VectorXd &xDetection, const VectorXd &xMeasPred, const MatrixXd &PMeasPred) {

    VectorXd mean(2);
    mean << 0, 0;

    return stats::dmvnorm<VectorXd, MatrixXd>((xDetection - xMeasPred), mean, PMeasPred, true);
}

double PDA::toPDF(const double &logPDF) {
    return exp(logPDF);
}

double PDA::getProbability(const double &logPDF) {

    double pdf = toPDF(logPDF);
    return (pdf * P_D) / spatialClutterIntensity;
}
