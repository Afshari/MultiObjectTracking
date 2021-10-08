#include "libs/murty/murty.h"

Murty::Murty(CostMatrix C) {
    double min = C.minCoeff();
    offset = 0;
    if (min < 0) {
        C.array() -= min;
        offset = min * C.rows();
    }
    queue.emplace(std::make_shared<MurtyState>(C));
}
