#ifndef MURTY_H
#define MURTY_H

#include <QObject>
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <algorithm>
#include <exception>
#include <iostream>
#include <memory>
#include <queue>
#include <set>
#include <tuple>
#include <vector>

#include "libs/murty/murty_funcs.h"
#include "libs/murty/murty_state.h"

class Murty : public QObject {
    Q_OBJECT


private:
   double offset;

public:
   explicit Murty(CostMatrix C);

   void get_partition_index(const MurtyState::Slacklist& partition_order,
                            MurtyState::Slacklist::iterator p, unsigned& i, unsigned& j) {
       i = std::get<1>(*p);
       j = std::get<2>(*p);
       for (auto pp = partition_order.begin(); pp != p; ++pp) {
           if (std::get<1>(*pp) < std::get<1>(*p)) { --i; }
           if (std::get<2>(*pp) < std::get<2>(*p)) { --j; }
       }
   }

   using ReturnTuple = std::tuple<bool, double, Assignment>;
   ReturnTuple draw_tuple() {
       Assignment sol;
       double cost;
       bool ok = draw(sol, cost);
       return {ok, cost, sol};
   }

   bool draw(Assignment& sol, double& cost) {
       std::shared_ptr<MurtyState> s;
       unsigned i, j;
       //std::cout << "Draw! Queue size: " << queue.size() << std::endl;

       if (queue.empty()) {
           return false;
       }

       for (s = queue.top(); !s->solved; s = queue.top()) {
           queue.pop();
           if (s->solve()) {
               queue.push(s);
           }
           if (queue.empty()) {
               //std::cout << "Queue empty 1!" << std::endl;
               return false;
           }
       }
       queue.pop();
       sol = s->solution;
       cost = s->cost + offset;
       if (cost > lap_inf) {
           return false;
       }
       //std::cout << "Solution: " << sol.transpose() << std::endl;
       //std::cout << "Cost: " << cost << std::endl;
       //std::cout << "res: " << s->res.transpose() << std::endl;
       //std::cout << "rmap: ";
       //for (auto& r : s->rmap) {
           //std::cout << r << ", ";
       //}
       //std::cout << std::endl;
       //std::cout << "cmap: ";
       //for (auto& c : s->cmap) {
           //std::cout << c << ", ";
       //}
       //std::cout << std::endl;
       //std::cout << s->C << std::endl;

       auto partition_order = s->minslack();
       auto p = partition_order.begin();
       auto node = s;

       for (; p != partition_order.end(); ++p) {
           get_partition_index(partition_order, p, i, j);
           queue.push(node->partition_without(i, j, std::get<0>(*p)));
           node = node->partition_with(i, j);
       }
       return true;
   }

private:
   std::priority_queue<MurtyStatePtr, std::vector<MurtyStatePtr>, MurtyStatePtrCompare> queue;
};

#endif // MURTY_H
