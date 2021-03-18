#include "measurementlineargaussian.h"

MeasurementLinearGaussian::MeasurementLinearGaussian() : state(4){

    state << 0, 1, 0, 1;

    std::cout << state << std::endl;

}
