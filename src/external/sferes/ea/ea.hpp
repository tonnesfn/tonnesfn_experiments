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




#ifndef EA_HPP_
#define EA_HPP_

#include <iostream>
#include <vector>
#include <sstream>
#include <iomanip>
#include <fstream>

#include <boost/fusion/container.hpp>
#include <boost/fusion/algorithm.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/fusion/support/is_sequence.hpp>
#include <boost/fusion/include/is_sequence.hpp>
#include <boost/fusion/algorithm/iteration/for_each.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <boost/mpl/joint_view.hpp>

#include "../dbg/dbg.hpp"
#include "../misc.hpp"
#include "../stc.hpp"
#include "../stat/state.hpp"

#include <chrono>
#include <ctime>

#ifndef VERSION
#define VERSION "version_unknown"
#endif

namespace sferes {
  namespace ea {

    template<typename E>
    struct RefreshStat_f {
      RefreshStat_f(const E &ea) : _ea(ea) {
      }
      const E& _ea;
      template<typename T>
      void operator() (T & x) const {
        x.refresh(_ea);
      }
    };
    template<typename A>
    struct WriteStat_f {
      WriteStat_f(A & a) : _archive(a) {
      }
      A& _archive;
      template<typename T>
      void operator() (const T &x) const {
        std::string version(VERSION);
        _archive << boost::serialization::make_nvp("version",
                 version);
        _archive << BOOST_SERIALIZATION_NVP(x);
      }
    };

    template<typename A>
    struct ReadStat_f {
      ReadStat_f(A & a) : _archive(a) {
      }
      A& _archive;
      template<typename T>
      void operator() (T & x) const {
        std::string version;
        _archive >> boost::serialization::make_nvp("version", version);
        if (version != std::string(VERSION))
          std::cerr << "WARNING: your are loading a file made with sferes version "
                    << version << " while the current version is:"
                    << VERSION
                    << std::endl;
        _archive >> BOOST_SERIALIZATION_NVP(x);
      }
    };

    struct ShowStat_f {
      ShowStat_f(unsigned n, std::ostream & os, size_t k) :
        _n(n), _i(0), _os(os), _k(k) {
      }
      template<typename T>
      void operator() (T & x) const {
        if (_i == _n)
          x.show(_os, _k);

        ++_i;
      }
      int _n;
      mutable int _i;
      std::ostream& _os;
      size_t _k;
    };


    template<typename E>
    struct ApplyModifier_f {
      ApplyModifier_f(E &ea) : _ea(ea) {
      }
      E& _ea;
      template<typename T>
      void operator() (T & x) const {
        x.apply(_ea);
      }
    };

    // we need this to resume only if there is a state
    // (and we need to be able to compile without a State)
    template<typename T, typename S>
    struct Resume {
      template<typename EA>
      void resume(EA& ea) {
        typedef sferes_stat::State<typename EA::phen_t, typename EA::params_t>  state_t;
        const state_t& s = *boost::fusion::find<state_t>(ea.stat());
        ea.set_gen(s.gen() + 1);
        ea.set_pop(s.pop());
      }
    };
    // do nothing if there is no state
    template<typename T>
    struct Resume<T, typename boost::fusion::result_of::end<T>::type> {
      template<typename EA>
      void resume(EA& ea) { }
    };

    template<typename Phen, typename Eval, typename Stat, typename FitModifier,
             typename Params,
             typename Exact = stc::Itself>
    class Ea : public stc::Any<Exact> {
     public:
      typedef Phen phen_t;
      typedef Eval eval_t;
      typedef Params params_t;

      // default behavior: we automatically add a State to the stats
      // define #SFERES_NO_STATE if you want to avoid this
      // (e.g. the population is too big)
#ifdef SFERES_NO_STATE
      typedef Stat stat_t;
#else
      typedef typename boost::fusion::vector<sferes_stat::State<Phen, Params> > state_v_t;
      typedef typename boost::fusion::joint_view<Stat, state_v_t> joint_t;
      typedef typename boost::fusion::result_of::as_vector<joint_t>::type  stat_t;
#endif

      typedef typename
      boost::mpl::if_<boost::fusion::traits::is_sequence<FitModifier>,
            FitModifier,
            boost::fusion::vector<FitModifier> >::type modifier_t;

      typedef std::vector<boost::shared_ptr<Phen> > pop_t;
      typedef typename phen_t::fit_t fit_t;
      Ea() : _pop(Params::pop::size), _gen(0), _stop(false) {
      }
      void set_fit_proto(const fit_t& fit) {
        _fit_proto = fit;
      }

      void saveCurrentDir(){
        FILE * currentDirFile;
        currentDirFile = fopen("currentEvoDir", "w");
        fprintf(currentDirFile, "%s", _res_dir.c_str());
        fclose(currentDirFile);
      }

      void run(std::string logPath, const std::string& exp_name = "") {
        dbg::trace trace("ea", DBG_HERE);
        _exp_name = exp_name;
        _make_res_dir();
        //saveCurrentDir();

        FILE *fp = fopen("generation", "w");
        fprintf(fp,"-1");
        fclose(fp);

        _set_status("running");
        random_pop();
        update_stats();
        write(-1);

        for (_gen = 0; _gen < Params::pop::nb_gen && !_stop; ++_gen)
          _iter();
        if (!_stop)
          _set_status("finished");
      }

      void resume(const std::string& fname) {
        dbg::trace trace("ea", DBG_HERE);
        _make_res_dir();
        //saveCurrentDir();
        _set_status("resumed");
        if (boost::fusion::find<sferes_stat::State<Phen, Params> >(_stat) == boost::fusion::end(_stat)) {
          std::cout<<"WARNING: no State found in stat_t, cannot resume" << std::endl;
          return;
        }
        _load(fname);
        typedef typename boost::fusion::result_of::find<stat_t, sferes_stat::State<Phen, Params> >::type has_state_t;
        Resume<stat_t, has_state_t> r;
        r.resume(*this);
        assert(!_pop.empty());
        std::cout<<"resuming at:"<< gen() << std::endl;
        for (; _gen < Params::pop::nb_gen && !_stop; ++_gen)
          _iter();
        if (!_stop)
          _set_status("finished");
      }
      void random_pop() {
        dbg::trace trace("ea", DBG_HERE);
        stc::exact(this)->random_pop();
      }
      void epoch() {
        dbg::trace trace("ea", DBG_HERE);
        stc::exact(this)->epoch();
      }
      // override _set_pop if you want to customize / add treatments
      // (in that case, DO NOT FORGET to add SFERES_EA_FRIEND(YouAlgo);)
      void set_pop(const pop_t& p) {
        dbg::trace trace("ea", DBG_HERE);
        this->_pop = p;
        for (size_t i = 0; i < this->_pop.size(); ++i)
          this->_pop[i]->develop();
        stc::exact(this)->_set_pop(p);
      }

      const pop_t& pop() const {
        return _pop;
      };
      pop_t& pop() {
        return _pop;
      };
      const eval_t& eval() const {
        return _eval;
      }
      eval_t& eval() {
        return _eval;
      }


      //---- modifiers ----
      const modifier_t& fit_modifier() const {
        return _fit_modifier;
      }
      template<int I>
      const typename boost::fusion::result_of::value_at_c<modifier_t, I>::type& fit_modifier() const {
        return boost::fusion::at_c<I>(_fit_modifier);
      }
      void apply_modifier() {
        boost::fusion::for_each(_fit_modifier, ApplyModifier_f<Exact>(stc::exact(*this)));
      }

      // ---- stats ---
      const stat_t& stat() const {
        return _stat;
      }
      template<int I>
      const typename boost::fusion::result_of::value_at_c<stat_t, I>::type& stat() const {
        return boost::fusion::at_c<I>(_stat);
      }
      void load(const std::string& fname) {
        _load(fname);
      }
      void show_stat(unsigned i, std::ostream& os, size_t k = 0) {
        boost::fusion::for_each(_stat, ShowStat_f(i, os, k));
      }
      void update_stats() {
        boost::fusion::for_each(_stat, RefreshStat_f<Exact>(stc::exact(*this)));
      }
      const std::string& res_dir() const {
        return _res_dir;
      }
      void set_res_dir(const std::string& new_dir) {
        _res_dir = new_dir;
        _make_res_dir();
      }
      size_t gen() const {
        return _gen;
      }
      void set_gen(unsigned g) {
        _gen = g;
      }
      size_t nb_evals() const {
          return _eval.nb_evals();
      }
      bool dump_enabled() const {
        return Params::pop::dump_period != -1;
      }
      void write() const {
        _write(gen());
      }
      void write(size_t g) const {
        _write(g);
      }
      void stop() {
        _stop = true;
        _set_status("interrupted");
      }
      bool is_stopped() const {
        return _stop;
      }

     protected:
      pop_t _pop;
      eval_t _eval;
      stat_t _stat;
      modifier_t _fit_modifier;
      std::string _res_dir;
      size_t _gen;
      fit_t _fit_proto;
      bool _stop;
      std::string _exp_name;

      void _iter() {
        FILE *fp = fopen("generation", "w");
        fprintf(fp,"%lu",_gen);
        fclose(fp);

        epoch();
        update_stats();
        if (_gen % Params::pop::dump_period == 0)
          _write(_gen);
      }

      // the status is a file that tells the state of the experiment
      // it is useful to tell to the rest of the world if the experiment has
      // been interrupted
      // typical values: "running", "interrupted", "finished"
      void _set_status(const std::string& status) const {
        std::string s = _res_dir + "/status";
        std::ofstream ofs(s.c_str());
        ofs << status;
      }

      template<typename P>
      void _eval_pop(P& p, size_t start, size_t end) {
        dbg::trace trace("ea", DBG_HERE);
        this->_eval.eval(p, start, end, this->_fit_proto);
      }
      // override _set_pop if you want to customize / add treatments
      // (in that case, DO NOT FORGET to add SFERES_EA_FRIEND(YouAlgo);)
      void _set_pop(const pop_t& p) {
        dbg::trace trace("ea", DBG_HERE);
      }
      void _make_res_dir() {
        dbg::trace trace("ea", DBG_HERE);
        if (Params::pop::dump_period == -1)
          return;
        if (_res_dir.empty()) {
          if (_exp_name.empty())
            _res_dir = misc::date() + "_" + misc::getpid();
          else
            _res_dir = _exp_name + "_" + misc::date() + "_" + misc::getpid();
	}
        boost::filesystem::path my_path(_res_dir);
        boost::filesystem::create_directory(my_path);
      }
      void _write(int gen) const {
        dbg::trace trace("ea", DBG_HERE);
        if (Params::pop::dump_period == -1)
          return;

        std::stringstream ss;

        if (gen != -1){
            ss << std::setw(4) << std::setfill('0') << gen;
        } else {
            ss << "-001";
        }

        std::string fname = _res_dir + std::string("/gen_")
                            + ss.str();


        std::ofstream ofs(fname.c_str());
#ifdef  SFERES_XML_WRITE
        typedef boost::archive::xml_oarchive oa_t;
#else
        typedef boost::archive::binary_oarchive oa_t;
#endif
        oa_t oa(ofs);
        boost::fusion::for_each(_stat, WriteStat_f<oa_t>(oa));
        std::cout << fname << " written" << std::endl;
      }
      void _load(const std::string& fname) {
        dbg::trace trace("ea", DBG_HERE);
        std::cout << "loading " << fname << std::endl;



        std::ifstream ifs(fname.c_str());
        if (ifs.fail()) {
          std::cerr << "Cannot open :" << fname
                    << "(does file exist ?)" << std::endl;
          exit(1);
        }
#ifdef SFERES_XML_WRITE
        typedef boost::archive::xml_iarchive ia_t;
#else
        typedef boost::archive::binary_iarchive ia_t;
#endif
        ia_t ia(ifs);
        boost::fusion::for_each(_stat, ReadStat_f<ia_t>(ia));
      }
    };
  }
}

#define SFERES_EA(Class, Parent)                                                               \
  template<typename Phen, typename Eval, typename Stat, typename FitModifier, typename Params, \
           typename Exact = stc::Itself>                                                       \
  class Class : public Parent < Phen, Eval, Stat, FitModifier, Params,                         \
  typename stc::FindExact<Class<Phen, Eval, Stat, FitModifier, Params, Exact>, Exact>::ret >

// useful to call protected functions of derived classes from the Ea
#define SFERES_EA_FRIEND(Class) \
      friend class Ea< Phen, Eval, Stat, FitModifier, Params,                         \
      typename stc::FindExact<Class<Phen, Eval, Stat, FitModifier, Params, Exact>, Exact>::ret >

#endif
