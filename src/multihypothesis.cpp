#include "inc/multihypothesis.h"

MultiHypothesis::MultiHypothesis(QList<SingleHypothesis *> *items, QObject *parent) : QObject(parent) {

    this->items = items;

}

void MultiHypothesis::normalizeWeights() {

//    std::cout << "normalize " << std::endl;
//    std::cout << items->length() << std::endl;

    double sum = 0;
    std::for_each(items->begin(), items->end(),
                    [&] (SingleHypothesis *item) {
                        sum += item->probability;
                    });

    std::for_each(items->begin(), items->end(),
                    [&] (SingleHypothesis *item) {
                        item->probability = item->probability / sum;
                    });

}
