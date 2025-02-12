//| This file is a part of the sferes2 framework.
//| Copyright 2009, ISIR / Universite Pierre et Marie Curie (UPMC)
//| Main contributor(s): Jean-Baptiste Mouret, mouret@isir.fr
//|
//| This software is a computer program whose purpose is to facilitate
//| experiments in evolutionary computation and evolutionary robotics.
//|
//| This software is governed by the CeCILL license under French law
//| and abiding by the rules of distribution of free software.  You
//| can use, modify and/ or redistribute the software under the terms
//| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
//| following URL "http://www.cecill.info".
//|
//| As a counterpart to the access to the source code and rights to
//| copy, modify and redistribute granted by the license, users are
//| provided only with a limited warranty and the software's author,
//| the holder of the economic rights, and the successive licensors
//| have only limited liability.
//|
//| In this respect, the user's attention is drawn to the risks
//| associated with loading, using, modifying and/or developing or
//| reproducing the software by the user in light of its specific
//| status of free software, that may mean that it is complicated to
//| manipulate, and that also therefore means that it is reserved for
//| developers and experienced professionals having in-depth computer
//| knowledge. Users are therefore encouraged to load and test the
//| software's suitability as regards their requirements in conditions
//| enabling the security of their systems and/or data to be ensured
//| and, more generally, to use and operate it in the same conditions
//| as regards security.
//|
//| The fact that you are presently reading this means that you have
//| had knowledge of the CeCILL license and that you accept its terms.




#ifndef EVAL_HPP_
#define EVAL_HPP_

bool sortReverse = false;

#include <vector>
#include <boost/shared_ptr.hpp>
#include "../dbg/dbg.hpp"
#include "../stc.hpp"

namespace sferes {
  namespace eval {
    SFERES_CLASS(Eval) {
    public:
      Eval() : _nb_evals(0) {}
      template<typename Phen>
      void eval(std::vector<boost::shared_ptr<Phen> >& pop, size_t begin, size_t end,
                const typename Phen::fit_t& fit_proto) {
        dbg::trace trace("eval", DBG_HERE);
        assert(pop.size());
        assert(begin < pop.size());
        assert(end <= pop.size());

        for (size_t i = begin; i < end; ++i) {
          pop[i]->fit() = fit_proto;
          pop[i]->develop();
        }

        // If morphology is included, sort:
        if (pop[0]->gen().size() > 7) {

          std::vector<std::vector<float>> legLengths;
          std::vector<int> sortedOrder;

          for (size_t i = begin; i < end; ++i) {
            auto individual = pop[i]->gen();
            std::vector<float> vec = {individual.data(0), individual.data(1)*4.0f}; // 4.0 from size diff of femur (25mm) and tibia (95mm)
            legLengths.push_back(vec);
          }

          std::vector<float> currentPosition = {0.0, 0.0};
          bool reachedPosition[legLengths.size()] = {0}; // Initialize to false

          for (int i = 0; i < legLengths.size(); i++) {

            float smallestDistance = 1000.0;
            int currentIndex = -1;

            for (int j = 0; j < legLengths.size(); j++){
              if (reachedPosition[j] == false){
                double currentDistance = std::max(fabs(legLengths[j][0] - currentPosition[0]), fabs(legLengths[j][1] - currentPosition[1]));
                if (currentDistance < smallestDistance) {
                  smallestDistance = currentDistance;
                  currentIndex = j;
                }
              }
            }

            reachedPosition[currentIndex] = true;
            currentPosition = legLengths[currentIndex];
            sortedOrder.push_back(currentIndex);
          }

          // Reverse order every second generation to save time reconfiguring
          if (sortReverse){
            std::reverse(std::begin(sortedOrder), std::end(sortedOrder));
          }

          sortReverse = !sortReverse;

          for (int i = 0; i < legLengths.size(); i++) {
            pop[begin+sortedOrder[i]]->fit().eval(*pop[begin+sortedOrder[i]]);
            _nb_evals++;
          }
        } else { // If morphology is NOT included, do unsorted:
          for (size_t i = begin; i < end; ++i) {
            pop[i]->fit().eval(*pop[i]);
            _nb_evals++;
          }
        }
      }
      unsigned nb_evals() const { return _nb_evals; }
    protected:
      unsigned _nb_evals;
    };
  }
}


#define SFERES_EVAL(Class, Parent)					\
  template <typename Params, typename Exact = stc::Itself> \
  class Class : public Parent<Params, typename stc::FindExact<Class<Params, Exact>, Exact>::ret>


#endif
