#ifndef TONNESFN_EXPERIMENTS_SFERES_NSGA2_H
#define TONNESFN_EXPERIMENTS_SFERES_NSGA2_H


namespace sferes_nsga2{
    using namespace sferes;
    using namespace sferes::gen::evo_float;

    struct Params {
        struct evo_float {
            SFERES_CONST float cross_rate = 0.0f;
            SFERES_CONST float mutation_rate = 1.0f;
            SFERES_CONST float sigma = 1.0f/6.0f;
            SFERES_CONST mutation_t mutation_type = gaussian;
            SFERES_CONST cross_over_t cross_over_type = recombination;
        };
        struct pop {
            SFERES_CONST unsigned size     =   popSize;  // Population size
            SFERES_CONST unsigned nb_gen   =   generations-1;  // Number of generations
            SFERES_CONST int dump_period   =    1;  // How often to save
            SFERES_CONST int initial_aleat =    1;  // Individuals to be created during random generation process
        };
        struct parameters {
            SFERES_CONST float min = 0.0f;
            SFERES_CONST float max = 1.0f;
        };
    };

    // Setup evolutionary framework
    typedef gen::EvoFloat<20, Params> gen_t;
    typedef phen::Parameters<gen_t,FitExp2MO<Params>, Params> phen_t;
    typedef eval::Eval<Params> eval_t;
    typedef boost::fusion::vector<sferes_stat::State<phen_t, Params> > stat_t;
    typedef modif::Dummy<> modifier_t;
    typedef ea::Nsga2<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;
    ea_t ea;
}

namespace sferes_mapElites{
    using namespace sferes;
    using namespace sferes::gen::evo_float;

    struct Params {
        struct ea {
            SFERES_CONST size_t behav_dim = 2;
            SFERES_CONST double epsilon = 0;//0.05;
            SFERES_ARRAY(size_t, behav_shape, 8, 8); // Define the size of the map
        };
        struct evo_float {
            SFERES_CONST float cross_rate = 0.0f;
            SFERES_CONST float mutation_rate = 1.0f;
            SFERES_CONST float sigma = 1.0f/6.0f;
            SFERES_CONST mutation_t mutation_type = gaussian;
            SFERES_CONST cross_over_t cross_over_type = recombination;
        };
        struct pop {
            SFERES_CONST size_t init_size  = popSize; // number of initial random points
            SFERES_CONST unsigned size     = popSize/2;  // Batch size
            SFERES_CONST unsigned nb_gen   = generations-1;  // Number of batches
            SFERES_CONST int dump_period   = 1;  // How often to save
            SFERES_CONST int initial_aleat = 1;  // Individuals to be created during random generation process
        };
        struct parameters {
            SFERES_CONST float min = 0.0f;
            SFERES_CONST float max = 1.0f;
        };
    };

    typedef gen::EvoFloat<10, Params> gen_t;
    typedef phen::Parameters<gen_t, FitMapElites<Params>, Params> phen_t;
    typedef eval::Eval<Params> eval_t;
    //typedef boost::fusion::vector<stat::State<phen_t, Params> > stat_t;
    typedef boost::fusion::vector< sferes_stat::Map<phen_t, Params>, sferes_stat::BestFit<phen_t, Params>, sferes_stat::MapBinary<phen_t, Params> > stat_t;
    typedef modif::Dummy<> modifier_t;
    //typedef ea::Nsga2<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;
    typedef ea::MapElites<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;
    ea_t ea;
}

#endif //TONNESFN_EXPERIMENTS_SFERES_NSGA2_H
