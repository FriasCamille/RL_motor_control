#include <vector>
#include <random>

class QLearning
{ 
    private:

        const size_t state_dimension;
        const size_t action_number;
        size_t state_number;
        double epsilon; 
        const double alpha;
        const double gamma;
        const double decay;
        std::vector<double> Qtable;
        std::vector<double> state_lower_limits;
        std::vector<double> state_upper_limits;
        std::vector<size_t> state_discretization;
        

        std::mt19937 gen;
        std::uniform_real_distribution<double> dis_eps;
        std::uniform_int_distribution<size_t> dis_action;

        //private methods

        size_t prox_value_iterator(double value, double upper, double lower, size_t disc);

    public:

        QLearning(size_t state_d, size_t action_n, double e, double a, double g, double d, std::vector<size_t> state_disc, std::vector<double> upper_limits, std::vector<double> lower_limits);

        //setters
        inline void set_table(std::vector<double> table){if(table.size() == Qtable.size()) Qtable = table;}

        //getters
        inline const std::vector<double>& get_table() const { return Qtable; }
        inline double get_epsilon(){return epsilon;}

        //methods
        inline double &Q(size_t s, size_t a) noexcept {return Qtable[s*action_number+a];}
        size_t max_action_index_Q(size_t s);
        inline size_t e_greedy(size_t s) noexcept{return (dis_eps(gen) < epsilon) ? dis_action(gen) : max_action_index_Q(s);}
        inline void decay_e() noexcept{epsilon *=decay; if(epsilon<0.01) epsilon = 0.01;}
        size_t get_state_index(const std::vector<double> &state);
        void update(size_t s, size_t a, double R, size_t s1) noexcept;


};